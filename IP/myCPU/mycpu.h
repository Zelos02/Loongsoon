`ifndef MYCPU_H
    `define MYCPU_H
    `define BR_BUS_WD       34
    // Simulation diagnostics are enabled automatically for RTL simulation but
    // are removed from the synthesis netlist.  This keeps the official routed
    // design identical to the functional M16.0F datapath while allowing every
    // NSCSCC performance test to print a machine-readable diagnosis block.
`ifdef SYNTHESIS
    `define DISABLE_PERF_COUNTERS
`else
    `define ENABLE_SIM_PERF_DIAG
    // The wide profiler is useful for one-off bottleneck captures but emits a
    // very large log.  Keep the compact counters for routine regressions.
    // `define ENABLE_M171W_WIDE_PROFILER
`endif
    // M17.5D retains the M17.4I 16KB/4-way ICache and expands the default
    // DCache from 16KB/4-way/256-set to 32KB/4-way/512-set.  The 32KB mode
    // uses inferred 512-deep block RAMs embedded in cache.v.  Rollback macros
    // allow independent ICache, DCache-capacity and HUR A/B experiments.
    // `define DISABLE_M175D_32KB_DCACHE
    // `define DISABLE_M174I_4WAY_ICACHE
    // `define DISABLE_M173N_HIT_UNDER_REFILL
    // M17.1W keeps the M17.0T functional datapath and adds a simulation-only
    // profiler for the eight lowest-scoring benchmarks.  ENABLE_M171W_WIDE_PROFILER
    // is defined only outside SYNTHESIS, so the routed design is unchanged.
    // M17.0T keeps the 16KB / 4-way DCache, but recovers timing by removing
    // the low-return reverse-lane cones from the default synthesis build.
    // Normal old-main + young-simple-ALU dual issue remains enabled.
    // DCache rollback remains available for strict timing/resource A/B.
    // `define DISABLE_M170C_4WAY_DCACHE
    // Define these opt-in macros only for controlled A/B experiments.
    // ENABLE_M17T_REVERSE_MEM_SWAP restores M16.1R ALU+memory reverse swap.
    // ENABLE_M17T_REVERSE_MUL_SWAP restores M16.2M ALU+multiply reverse swap.
`ifndef ENABLE_M17T_REVERSE_MEM_SWAP
    `define DISABLE_M16_LANE_SWAP
`endif
`ifndef ENABLE_M17T_REVERSE_MUL_SWAP
    `define DISABLE_M162_ALU_MUL_SWAP
`endif
    // Frontend performance default:
    // Diagnostic data showed that the 4-entry IF queue was full for most cycles
    // while second-slot branches caused a large fraction of all redirects.  Keep
    // the backend dual-issue machinery, but return the default frontend to one
    // instruction per accepted ICache request.  Comment this line only for A/B.
    // M18.1 gives PC+4 an independent BTB/BHT lookup before dual fetch is
    // enabled.  Defining the legacy rollback macro removes the second lookup
    // after synthesis and returns to the proven one-word frontend.
    // M19.1 enables the widened 8-entry IFQ and dual-word ICache requests.
    // The queue keeps both words of a fetch bundle resident across backend
    // stalls, so dual fetch is no longer immediately throttled by a 4-entry
    // buffer. Comment this line for the BTB512 single-fetch rollback.
    // Controlled A/B switch: restore the exact pre-M18.0 refill behavior and
    // pre-M18.2 issue policy without touching any project/constraint setting.
    // Keep this commented for the optimized submission build.
    // `define ENABLE_M18_BASELINE_AB
`ifdef ENABLE_M18_BASELINE_AB
    `define DISABLE_M180_ICACHE_LINE_FILL
    `define DISABLE_M180B_ADJACENT_LINE_FILL
    `define DISABLE_M182_SLOT1_BRANCH_PAIR
    `define DISABLE_M183_QUAD_ICACHE_FILL
`endif
    // M18.2 lets a younger conditional branch or direct B instruction resolve
    // beside an older non-control instruction.  Define this only for rollback.
    `define DISABLE_M182_SLOT1_BRANCH_PAIR
    // M18.0B merges a demand DCache line and its ascending neighbour into one
    // 8-beat AXI read.  It is enabled by default because the competition RAM
    // model charges its long delay once per read transaction.  Define the
    // rollback macro below for a legacy 4-beat/16-byte refill A/B build.
    // `define DISABLE_M180B_ADJACENT_LINE_FILL
    // M18.3 extends each merged refill from three to five cache lines.  Define
    // this rollback macro to retain the proven 12-beat M18.0 transaction.
    // CPU-side ARLEN is 4-bit AXI3 in the released SoC, so bursts longer than
    // 16 beats are not legal even though the myCPU wrapper exposes 8 bits.
    `define DISABLE_M183_FIVE_LINE_FILL
    // M18.3Q keeps the legal maximum 16-beat ICache refill enabled.  Define
    // this rollback macro to use the previous 12-beat/three-line ICache.
    // `define DISABLE_M183_QUAD_ICACHE_FILL
    // M18.4 grows only the DCache to 1024 sets x 4 ways x 16 bytes = 64KB.
    // This contains the 40KB Fireye A0 working set. Define for 32KB rollback.
    // `define DISABLE_M184_64KB_DCACHE
    // M18.5 removes the otherwise mandatory one-cycle ID address-capture
    // bubble for ordinary DMW load/store accesses.  ID carries only the
    // selected three-bit physical segment into EXE; EXE combines it with the
    // already-forwarded rj+imm effective address.  Real TLB accesses and
    // misalignment exceptions retain the registered precheck protocol.
    // `define DISABLE_M185_FAST_DMW
    // M18.6 allows the DCache to accept a new request in the same cycle that
    // the previous lookup hits.  This changes the hit path from one request
    // every two cycles to one request per cycle for independent accesses.
    // Define only to return to the legacy non-pipelined DCache.
    // Exact 20-test A/B showed no cycle-count change because the current
    // in-order MEM stage does not present a younger request early enough to
    // use the cache's same-cycle hit window.  Keep the lower-risk/timing-clean
    // single-port schedule until the MEM stage itself is decoupled.
    `define DISABLE_M186_PIPELINED_DCACHE
    // M18.7 retires ordinary cached stores once they are safely held by the
    // ordered DCache request path.  Their later cache-completion responses are
    // counted and discarded, while loads, uncached stores and CACOP operations
    // retain the original completion semantics.
    // `define DISABLE_M187_EARLY_STORE_ACK
    // IF -> ID: {slot0_dual_static_ok, pred_taken, pred_nextpc, inst, pc}
    `define FS_TO_DS_BUS_WD 98
    // IF-queue candidate predecode metadata:
    // {simple, mem, load, store, src1_used, src2_used, src2_is_rd,
    //  src1_is_pc, imm_sel[1:0], alu_op[11:0], mem_op[7:0]}
    `define SLOT1_META_WD 30
    // M17.8B defers only conditional branches that consume the current
    // main-EXE load.  The load value is forwarded from MEM exclusively into
    // the EXE branch comparator; it is never exposed to ID.  Define the
    // rollback macro below for an exact M17.5D-style branch-hazard A/B build.
    // `define DISABLE_M178B_DEFERRED_LOAD_BRANCH
    // One program-age bit is carried down the main pipeline so a swapped
    // pair can still be reported/committed in architectural order.  M17.8B
    // adds 37 registered bits for deferred-branch metadata:
    // {defer, rj_from_load, rd_from_load, pred_taken,
    //  pred_target_miss, taken_target[31:0]}.
    `define DS_TO_ES_BUS_WD 315
    `define ES_TO_MS_BUS_WD 110
    `define MS_TO_WS_BUS_WD 103
    `define WS_TO_RF_BUS_WD 38
`endif
