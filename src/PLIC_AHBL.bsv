// Copyright (c) 2019-2020 Bluespec, Inc.  All Rights Reserved

package PLIC_AHBL;

// ================================================================
// This package implements a PLIC (Platform-Level Interrupt Controller)
// conforming to the RISC-V PLIC standard that integrates with a
// AHBL system.
// It is parameterized for:
//   - # of sources
//   - # of targets
//   - # of priorities
//
// ================================================================
// Bluespec lib imports

import  ConfigReg    :: *;
import  Vector       :: *;
import  ClientServer :: *;
import  Assert       :: *;

// ----------------
// BSV additional libs

import  Cur_Cycle    :: *;
import  GetPut_Aux   :: *;

// ================================================================
// Project imports

import AHBL_Types    :: *;
import AHBL_Defs     :: *;
import Fabric_Defs   :: *;    // for Wd_Id, Wd_Addr, Wd_Data, Wd_User

// ================================================================
// Change bitwidth without requiring < or > constraints.

function Bit #(m) changeWidth (Bit #(n) x);
   Bit #(TAdd #(m, n)) y = zeroExtend (x);
   Bit #(m)            z = y [valueOf (m)-1 : 0];
   return z;
endfunction

// ================================================================
// Maximum supported sources, targets, ...

typedef  10  T_wd_source_id;    // Max 1024 sources (source 0 is reserved for 'no interrupt')
typedef   5  T_wd_target_id;    // Max 32 targets
typedef enum { RST, RDY, RSP, ERR1, ERR2 } AHBL_Tgt_State deriving (Bits, Eq, FShow);

// ================================================================
// Interfaces

// ----------------
// Individual source interface

interface PLIC_Source_IFC;
   (* always_ready, always_enabled *)
   method Action  m_interrupt_req (Bool set_not_clear);
endinterface

// ----------------
// Individual target interface

interface PLIC_Target_IFC;
   (* always_ready *)
   method Bool  m_eip;    // external interrupt pending
endinterface

// ----------------
// PLIC interface

interface PLIC_IFC #(numeric type  t_n_external_sources,
                     numeric type  t_n_targets,
                     numeric type  t_max_priority);

   // Main Fabric Reqs/Rsps
   interface AHBL_Slave_IFC #(AHB_Wd_Data) fabric;

   // sources
   interface Vector #(t_n_external_sources, PLIC_Source_IFC)  v_sources;

   // targets EIPs (External Interrupt Pending)
   interface Vector #(t_n_targets, PLIC_Target_IFC) v_targets;
endinterface

// ================================================================
// PLIC module implementation

module mkPLIC #(
   function Tuple2 #(Bit #(t_wd_priority), Bit #(TLog #(t_n_sources))
   ) fn_target_max_prio_and_max_id0 (
        Vector #(t_n_sources, Bool)                         vrg_source_ip
      , Vector #(t_n_targets, Vector #(t_n_sources, Bool))  vvrg_ie
      , Vector #(t_n_sources, Bit #(t_wd_priority))         vrg_source_prio
      , Bit    #(T_wd_target_id)  target_id)
) (PLIC_IFC #(t_n_external_sources, t_n_targets, t_max_priority)) provisos (
     Add #(1, t_n_external_sources, t_n_sources)   // source 0 is reserved for 'no source'
   , Add #(_any_0, TLog #(t_n_sources), T_wd_source_id)
   , Add #(_any_1, TLog #(t_n_targets), T_wd_target_id)
   , Log #(TAdd #(t_max_priority, 1), t_wd_priority));

   // 0 = quiet; 1 = show PLIC transactions; 2 = also show AXI4 transactions
   Bit #(2) verbosity = 0;

   // Source_Ids and Priorities are read and written over the memory interface
   // and should fit within the data bus width, currently 64 bits.
   staticAssert ((valueOf (TLog #(t_n_sources))               <= 64), "PLIC: t_n_sources parameter too large");
   staticAssert ((valueOf (TLog #(TAdd #(t_max_priority, 1))) <= 64), "PLIC: t_max_priority parameter too large");

   Integer  n_sources = valueOf (t_n_sources);
   Integer  n_targets = valueOf (t_n_targets);

   // ----------------
   // Memory-mapped access

   // ----------------
   // AHB-Lite signals and registers

   // Inputs
   Wire #(Bool)            w_hsel      <- mkBypassWire;
   Wire #(Bool)            w_hready_in <- mkBypassWire;
   Wire #(AHB_Fabric_Addr) w_haddr     <- mkBypassWire;
   Wire #(AHBL_Burst)      w_hburst    <- mkBypassWire;
   Wire #(Bool)            w_hmastlock <- mkBypassWire;
   Wire #(AHBL_Prot)       w_hprot     <- mkBypassWire;
   Wire #(AHBL_Size)       w_hsize     <- mkBypassWire;
   Wire #(AHBL_Trans)      w_htrans    <- mkBypassWire;
   Wire #(AHB_Fabric_Data) w_hwdata    <- mkBypassWire;
   Wire #(Bool)            w_hwrite    <- mkBypassWire;

   // Outputs
   Reg  #(Bool)            rg_hready    <- mkReg(True);
   Reg  #(AHBL_Resp)       rg_hresp     <- mkReg(AHBL_OKAY);
   Reg  #(AHB_Fabric_Data) rg_hrdata    <- mkRegU;

   Reg #(AHB_Fabric_Addr)  rg_haddr     <- mkRegU;
   Reg #(AHBL_Size)        rg_hsize     <- mkRegU;
   Reg #(AHBL_Trans)       rg_htrans    <- mkRegU;
   Reg #(Bool)             rg_hwrite    <- mkRegU;

   Reg #(AHBL_Tgt_State)   rg_state  <- mkReg (RST);

   // ----------------
   // Per-interrupt source state

   // Interrupt pending from source
   Vector #(t_n_sources, Reg #(Bool))                  vrg_source_ip   <- replicateM (mkConfigReg (False));
   // Interrupt claimed and being serviced by a hart
   Vector #(t_n_sources, Reg #(Bool))                  vrg_source_busy <- replicateM (mkReg (False));
   // Priority for this source
   Vector #(t_n_sources, Reg #(Bit #(t_wd_priority)))  vrg_source_prio <- replicateM (mkReg (0));

   // ----------------
   // Per-target hart context state

   // Threshold: interrupts at or below threshold should be masked out for target
   Vector #(t_n_targets, Reg #(Bit #(t_wd_priority)))   vrg_target_threshold <- replicateM (mkReg ('1));
   // Target has claimed interrupt for source and is servicing it
   Vector #(t_n_targets, Reg #(Bit #(TLog #(t_n_sources))))  vrg_servicing_source <- replicateM (mkReg (0));

   // ----------------
   // Per-target, per-source state

   // Interrupt enables from source to target
   Vector #(t_n_targets,
            Vector #(t_n_sources, Reg #(Bool)))  vvrg_ie <- replicateM (replicateM (mkReg (False)));

   // Memory map
   // Address mask for PLIC addresses (from SoC Map limit)
   Fabric_Addr addr_mask = 'h3f_ffff;

   // ================================================================
   // Compute outputs for each target (combinational)

   let fn_target_max_prio_and_max_id = fn_target_max_prio_and_max_id0(readVReg(vrg_source_ip),
                                                                      map(readVReg, vvrg_ie),
                                                                      readVReg(vrg_source_prio));

   function Action fa_show_PLIC_state;
      action
         $display ("----------------");
         $write ("Src IPs  :");
         for (Integer source_id = 0; source_id < n_sources; source_id = source_id + 1)
            $write (" %0d", pack (vrg_source_ip [source_id]));
         $display ("");

         $write ("Src Prios:");
         for (Integer source_id = 0; source_id < n_sources; source_id = source_id + 1)
            $write (" %0d", vrg_source_prio [source_id]);
         $display ("");

         $write ("Src busy :");
         for (Integer source_id = 0; source_id < n_sources; source_id = source_id + 1)
            $write (" %0d", pack (vrg_source_busy [source_id]));
         $display ("");

         for (Integer target_id = 0; target_id < n_targets; target_id = target_id + 1) begin
            $write ("T %0d IEs  :", target_id);
            for (Integer source_id = 0; source_id < n_sources; source_id = source_id + 1)
               $write (" %0d", vvrg_ie [target_id][source_id]);
            match { .max_prio, .max_id } = fn_target_max_prio_and_max_id (fromInteger (target_id));
            $display (" MaxPri %0d, Thresh %0d, MaxId %0d, Svcing %0d",
                      max_prio, vrg_target_threshold [target_id], max_id, vrg_servicing_source [target_id]);
         end
      endaction
   endfunction

   // ----------------
   // Address Checks
   // Is the address okay? Use the raw address from the bus as this check is done
   // in the first phase.
   let addr_is_ok = fn_ahbl_is_aligned (w_haddr[1:0], w_hsize);
   
   // ================================================================
   // Reset

   rule rl_reset (rg_state == RST);
      if (verbosity != 0)
         $display ("%6d:[D]:%m.rl_reset", cur_cycle);

      for (Integer source_id = 0; source_id < n_sources; source_id = source_id + 1) begin
         vrg_source_ip   [source_id] <= False;
         vrg_source_busy [source_id] <= False;
         vrg_source_prio [source_id] <= 0;
      end

      for (Integer target_id = 0; target_id < n_targets; target_id = target_id + 1) begin
         // Mask all interrupts with highest threshold
         vrg_target_threshold [target_id] <= '1;
         vrg_servicing_source [target_id] <=  0;
      end

      for (Integer target_id = 0; target_id < n_targets; target_id = target_id + 1)
         for (Integer source_id = 0; source_id < n_sources; source_id = source_id + 1)
            vvrg_ie [target_id][source_id] <= False;

      // ready the fabric interface ...
      rg_state     <= RDY;
      rg_hready    <= True;
      rg_hresp     <= AHBL_OKAY;
   endrule

   // ================================================================
   // Bus interface for reading/writing control/status regs
   // Relative-address map is same as 'SiFive U54-MC Core Complex Manual v1p0'.
   // Accesses are 4-bytes wide, even though bus may be 64b wide.
   
   function Action fa_ahbl_error;
      action
         rg_state    <= ERR1;
         rg_hready   <= False;
         rg_hresp    <= AHBL_ERROR;
      endaction
   endfunction 

   (* fire_when_enabled, no_implicit_conditions *)
   rule rl_new_req (   (rg_state == RDY)
                    && (w_hsel && w_hready_in && (w_htrans == AHBL_NONSEQ)));

      // Register fresh address-and-control inputs
      rg_haddr    <= w_haddr;
      rg_hsize    <= w_hsize;
      rg_htrans   <= w_htrans;
      rg_hwrite   <= w_hwrite;


      if (addr_is_ok) begin
         rg_state    <= RSP;
         rg_hresp    <= AHBL_OKAY;

         // Immediate responses are not possible because error
         // checking requires an extra cycle for timing reasons
         rg_hready   <= False;
      end

      // Error case (two cycle response)
      else begin
         fa_ahbl_error;
      end

      if (verbosity != 0)
         $display ("%06d:[D]:%m.rl_new_req: haddr 0x%08h", cur_cycle,
            w_haddr, fshow (w_hsize), " hwrite %0d htrans ", w_hwrite, fshow (w_htrans));
   endrule

   // ----------------------------------------------------------------
   // Handle memory-mapped write requests

   let addr_offset = rg_haddr & addr_mask;
   rule rl_wr_req ((rg_state == RSP) && (rg_hwrite));
      // Writes
      Bool werr = False; 
      // Source priority
      if (addr_offset < 'h1000) begin
         Bit #(T_wd_source_id)  source_id = truncate (addr_offset [11:2]);
         // Note: write to source_id 0 is error; should it just be ignored?
         if (source_id == 0) werr = True;
         else if (source_id <= fromInteger (n_sources - 1)) begin
            vrg_source_prio [source_id] <= changeWidth (w_hwdata);

            if (verbosity > 0)
               $display ("%0d: PLIC.rl_wr_req: writing Source Priority: source %0d = 0x%0h",
                         cur_cycle, source_id, w_hwdata);
         end
         else werr = True;
      end

      // Source IPs (interrupt pending).
      // Read-only, so ignore write; just check that addr ok.
      else if (('h1000 <= addr_offset) && (addr_offset < 'h2000)) begin
         Bit #(T_wd_source_id)  source_id_base = truncate ({ addr_offset [11:0], 5'h0 });

         if (source_id_base <= fromInteger (n_sources - 1)) begin
            if (verbosity > 0)
               $display ("%0d: PLIC.rl_wr_req: Ignoring write to Read-only Intr Pending 32 bits from source %0d",
                         cur_cycle, source_id_base);
         end
         else  werr = True;
      end

      // Source IEs (interrupt enables) for a target
      // Write 32 consecutive IE bits starting with addr.
      // Target 0 addrs: 2000-207F, Target 1 addrs: 2080-20FF, ...
      else if (('h2000 <= addr_offset) && (addr_offset < 'h3000)) begin
         Bit #(T_wd_target_id)  target_id      = truncate (addr_offset [11:7]);
         Bit #(T_wd_source_id)  source_id_base = truncate ({ addr_offset [6:0], 5'h0 });

         if (   (source_id_base <= fromInteger (n_sources - 1))
             && (target_id      <= fromInteger (n_targets - 1))) begin
            for (Bit #(T_wd_source_id)  k = 0; k < 32; k = k + 1) begin
               Bit #(T_wd_source_id)  source_id = source_id_base + k;
               if (source_id <= fromInteger (n_sources - 1))
                  vvrg_ie [target_id][source_id] <= unpack (w_hwdata [k]);
            end

            if (verbosity > 0)
               $display ("%0d: PLIC.rl_wr_req: writing Intr Enable 32 bits for target %0d from source %0d = 0x%0h",
                         cur_cycle, target_id, source_id_base, w_hwdata);
         end
         else  werr = True;
      end

      // Target threshold
      else if ((addr_offset [31:0] & 32'hFFFF_0FFF) == 32'h0020_0000) begin
         Bit #(T_wd_target_id)  target_id = truncate (addr_offset [20:12]);
         if (target_id <= fromInteger (n_targets - 1)) begin
            vrg_target_threshold [target_id] <= changeWidth (w_hwdata);

            if (verbosity > 0)
               $display ("%0d: PLIC.rl_wr_req: writing threshold for target %0d = 0x%0h",
                         cur_cycle, target_id, w_hwdata);
         end
         else  werr = True;
      end

      // Interrupt service completion by target
      // Actual memory-write-data is irrelevant.
      else if ((addr_offset [31:0] & 32'hFFFF_0FFF) == 32'h0020_0004) begin
         Bit #(T_wd_target_id)  target_id = truncate (addr_offset [20:12]);
         Bit #(T_wd_source_id)  source_id = zeroExtend (vrg_servicing_source [target_id]);

         if (target_id <= fromInteger (n_targets - 1)) begin
            if (vrg_source_busy [source_id]) begin
               vrg_source_busy [source_id] <= False;
               vrg_servicing_source [target_id] <= 0;
               if (verbosity > 0)
                  $display ("%06d:[D]:%m.rl_wr_req: writing completion for target %0d for source 0x%0h",
                     cur_cycle, target_id, source_id);
            end
            else begin
               $display ("%06d:[E]:%m.rl_wr_req: interrupt completion to source that is not being serviced",
                         cur_cycle);
               $display ("    Completion message from target %0d to source %0d", target_id, source_id);
               $display ("    Ignoring");
               werr = True;
            end
         end
         else  werr = True;
      end

      else begin
         werr = True;
         $display ("%06d:[E]:%m.rl_wr_req: unrecognized addr: %08h", cur_cycle, addr_offset);
      end

      if (werr) fa_ahbl_error;
      else begin
         rg_hready <= True;
         rg_state  <= RDY;
      end

      if (verbosity > 1) 
         $display ("%06d:[D]:%m.rl_wr_req", cur_cycle);
   endrule

   // ----------------------------------------------------------------
   // Handle memory-mapped read requests
   rule rl_rd_req ((rg_state == RSP) && (!rg_hwrite));
      // Reads
      Bool rerr = False; 
      Bit #(32) rdata = ?;

      // Source Priority
      if (addr_offset < 'h1000) begin
         Bit #(T_wd_source_id)  source_id = truncate (addr_offset [11:2]);
         if (source_id == 0) rerr = True;
         else if (source_id <= fromInteger (n_sources - 1)) begin
            rdata = changeWidth (vrg_source_prio [source_id]);

            if (verbosity > 0)
               $display ("%0d: PLIC.rl_process_rd_req: reading Source Priority: source %0d = 0x%0h",
                         cur_cycle, source_id, rdata);
         end
         else rerr = True;
      end

      // Source IPs (interrupt pending).
      // Return 32 consecutive IP bits starting with addr.
      else if (('h1000 <= addr_offset) && (addr_offset < 'h2000)) begin
         Bit #(T_wd_source_id)  source_id_base = truncate ({ addr_offset [11:0], 5'h0 });

         function Bool fn_ip_source_id (Integer source_id_offset);
            let source_id = source_id_base + fromInteger (source_id_offset);
            Bool ip_source_id = (  (source_id <= fromInteger (n_sources - 1))
                                 ? vrg_source_ip [source_id]
                                 : False);
            return ip_source_id;
         endfunction

         if (source_id_base <= fromInteger (n_sources - 1)) begin
            Bit #(32) v_ip = pack (genWith  (fn_ip_source_id));
            rdata = changeWidth (v_ip);

            if (verbosity > 0)
               $display ("%0d: PLIC.rl_process_rd_req: reading Intr Pending 32 bits from source %0d = 0x%0h",
                         cur_cycle, source_id_base, rdata);
         end
         else rerr = True;
      end

      // Source IEs (interrupt enables) for a target
      // Return 32 consecutive IE bits starting with addr.
      // Target 0 addrs: 2000-207F, Target 1 addrs: 2080-20FF, ...
      else if (('h2000 <= addr_offset) && (addr_offset < 'h3000)) begin
         Bit #(T_wd_target_id)  target_id      = truncate (addr_offset [11:7]);
         Bit #(T_wd_source_id)  source_id_base = truncate ({ addr_offset [6:0], 5'h0 });

         function Bool fn_ie_source_id (Integer source_id_offset);
            let source_id = fromInteger (source_id_offset) + source_id_base;
            return (  (source_id <= fromInteger (n_sources - 1))
                    ? vvrg_ie [target_id][source_id]
                    : False);
         endfunction

         if (   (source_id_base <= fromInteger (n_sources - 1))
             && (target_id      <= fromInteger (n_targets - 1))) begin
            Bit #(32) v_ie = pack (genWith  (fn_ie_source_id));
            rdata = changeWidth (v_ie);

            if (verbosity > 0)
               $display ("%0d: PLIC.rl_process_rd_req: reading Intr Enable 32 bits from source %0d = 0x%0h",
                         cur_cycle, source_id_base, rdata);
         end
         else rerr = True;
      end

      // Target threshold
      else if ((addr_offset [31:0] & 32'hFFFF_0FFF) == 32'h0020_0000) begin
         Bit #(T_wd_target_id)  target_id = truncate (addr_offset [20:12]);
         if (target_id <= fromInteger (n_targets - 1)) begin
            rdata = changeWidth (vrg_target_threshold [target_id]);

            if (verbosity > 0)
               $display ("%0d: PLIC.rl_process_rd_req: reading Threshold for target %0d = 0x%0h",
                         cur_cycle, target_id, rdata);
         end
         else rerr = True;
      end

      // Interrupt service claim by target
      else if ((addr_offset [31:0] & 32'hFFFF_0FFF) == 32'h0020_0004) begin
         Bit #(T_wd_target_id)  target_id  = truncate (addr_offset [20:12]);
         match { .max_prio, .max_id } = fn_target_max_prio_and_max_id (target_id);
         Bool eip = (max_prio > vrg_target_threshold [target_id]);
         if (target_id <= fromInteger (n_targets - 1)) begin
            if (vrg_servicing_source [target_id] != 0) begin
               $display ("%0d: ERROR: %m.rl_rd_req: target %0d claiming without prior completion",
                         cur_cycle, target_id);
               $display ("    Still servicing interrupt from source %0d", vrg_servicing_source [target_id]);
               $display ("    Trying to claim service   for  source %0d", max_id);
               $display ("    Ignoring.");
               rerr = True;
            end
            else begin
               if (max_id != 0) begin
                  vrg_source_ip   [max_id]         <= False;
                  vrg_source_busy [max_id]         <= True;
                  vrg_servicing_source [target_id] <= truncate (max_id);
                  rdata = changeWidth (max_id);

                  if (verbosity > 0)
                     $display ("%0d: PLIC.rl_process_rd_req: reading Claim for target %0d = 0x%0h",
                        cur_cycle, target_id, rdata);
               end
            end
         end
         else rerr = True;
      end

      else begin
         rerr = True;
         $display ("%06d:[E]:%m.rl_rd_req: unrecognized addr: 0x%08h"
            , cur_cycle, addr_offset);
      end

      if (rerr) fa_ahbl_error;
      else begin
         rg_hready <= True;
         rg_state  <= RDY;
         rg_hrdata <= rdata;
      end

   endrule

   rule rl_idle (   (rg_state == RDY)
                 && (w_hsel && (w_htrans == AHBL_IDLE)));
      rg_hready <= True;
   endrule

   rule rl_error1 (rg_state == ERR1);
      rg_state  <= ERR2;
      rg_hready <= True;
   endrule

   rule rl_error2 (rg_state == ERR2);
      rg_state  <= RDY;
      rg_hready <= True;
      rg_hresp  <= AHBL_OKAY;
   endrule

   // ================================================================
   // Creator of each source interface

   function PLIC_Source_IFC  fn_mk_PLIC_Source_IFC (Integer source_id);
      return interface PLIC_Source_IFC;
          method Action  m_interrupt_req (Bool set_not_clear);
             action
                if (! vrg_source_busy [source_id + 1]) begin
                   vrg_source_ip [source_id + 1] <= set_not_clear;

                   if ((verbosity > 0) && (vrg_source_ip [source_id + 1] != set_not_clear))
                      $display ("%0d: %m.m_interrupt_req: changing vrg_source_ip [%0d] to %0d",
                                cur_cycle, source_id + 1, pack (set_not_clear));
                end
             endaction
          endmethod
      endinterface;
   endfunction

   // ================================================================
   // Creator of each target interface

   function PLIC_Target_IFC  fn_mk_PLIC_Target_IFC (Integer target_id);
      return interface PLIC_Target_IFC;
          method Bool m_eip;    // external interrupt pending
             match { .max_prio, .max_id } = fn_target_max_prio_and_max_id (fromInteger (target_id));
             Bool eip = (max_prio > vrg_target_threshold [target_id]);
             return eip;
          endmethod
      endinterface;
   endfunction

   // ================================================================
   // INTERFACE

   // sources
   interface  v_sources    = genWith  (fn_mk_PLIC_Source_IFC);

   // targets
   interface  v_targets    = genWith  (fn_mk_PLIC_Target_IFC);

   // Memory-mapped access: fabric interface
   interface AHBL_Slave_IFC fabric;
      // ----------------
      // Inputs

      method Action hsel (Bool sel);
         w_hsel <= sel;
      endmethod

      method Action hready (Bool hready_in);
         w_hready_in <= hready_in;
      endmethod

      method Action haddr (AHB_Fabric_Addr addr);
         w_haddr <= addr;
      endmethod

      method Action hburst (AHBL_Burst burst);
         w_hburst <= burst;
      endmethod

      method Action hmastlock (Bool mastlock);
         w_hmastlock <= mastlock;
      endmethod

      method Action hprot (AHBL_Prot prot);
         w_hprot <= prot;
      endmethod

      method Action hsize (AHBL_Size size);
         w_hsize <= size;
      endmethod

      method Action htrans (AHBL_Trans trans);
         w_htrans <= trans;
      endmethod

      method Action hwdata(AHB_Fabric_Data data);
         w_hwdata <= data;
      endmethod

      method Action hwrite (Bool write);
         w_hwrite <= write;
      endmethod

      // ----------------
      // Outputs

      method Bool             hreadyout = rg_hready;
      method AHBL_Resp        hresp     = rg_hresp;
      method AHB_Fabric_Data  hrdata    = rg_hrdata;
   endinterface
endmodule

// ================================================================

endpackage
