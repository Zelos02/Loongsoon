module alu(
  input  wire [11:0] alu_op,
  input  wire [31:0] alu_src1,
  input  wire [31:0] alu_src2,
  output wire [31:0] alu_result
);

wire op_add;   // add operation
wire op_sub;   // sub operation
wire op_slt;   // signed compare and set less than
wire op_sltu;  // unsigned compare and set less than
wire op_and;   // bitwise and
wire op_nor;   // bitwise nor
wire op_or;    // bitwise or
wire op_xor;   // bitwise xor
wire op_sll;   // logical left shift
wire op_srl;   // logical right shift
wire op_sra;   // arithmetic right shift
wire op_lui;   // load upper immediate

assign op_add  = alu_op[ 0];
assign op_sub  = alu_op[ 1];
assign op_slt  = alu_op[ 2];
assign op_sltu = alu_op[ 3];
assign op_and  = alu_op[ 4];
assign op_nor  = alu_op[ 5];
assign op_or   = alu_op[ 6];
assign op_xor  = alu_op[ 7];
assign op_sll  = alu_op[ 8];
assign op_srl  = alu_op[ 9];
assign op_sra  = alu_op[10];
assign op_lui  = alu_op[11];

// A single FPGA carry chain handles ADD/SUB/SLT/SLTU.  This avoids creating
// separate signed/unsigned comparator cones for slt/sltu.
wire        need_sub;
wire [31:0] adder_b;
wire        adder_cin;
wire [31:0] adder_result;
wire        adder_cout;

assign need_sub  = op_sub | op_slt | op_sltu;
assign adder_b   = need_sub ? ~alu_src2 : alu_src2;
assign adder_cin = need_sub ? 1'b1      : 1'b0;
assign {adder_cout, adder_result} = alu_src1 + adder_b + adder_cin;

wire        slt_less;
wire        sltu_less;
wire [31:0] add_sub_result;
wire [31:0] slt_result;
wire [31:0] sltu_result;
wire [31:0] and_result;
wire [31:0] or_result;
wire [31:0] nor_result;
wire [31:0] xor_result;
wire [31:0] lui_result;
wire [31:0] sll_result;
wire [31:0] srl_result;
wire [31:0] sra_result;
wire [ 4:0] shamt;

assign add_sub_result = adder_result;
assign slt_less       = (alu_src1[31] ^ alu_src2[31]) ? alu_src1[31] : adder_result[31];
assign sltu_less      = ~adder_cout;
assign slt_result     = {31'b0, slt_less};
assign sltu_result    = {31'b0, sltu_less};

assign and_result = alu_src1 & alu_src2;
assign or_result  = alu_src1 | alu_src2;
assign nor_result = ~or_result;
assign xor_result = alu_src1 ^ alu_src2;
assign lui_result = alu_src2;

assign shamt      = alu_src2[4:0];
assign sll_result = alu_src1 <<  shamt;
assign srl_result = alu_src1 >>  shamt;
assign sra_result = $signed(alu_src1) >>> shamt;

// alu_op is generated as one-hot by ID.  The masked-OR mux maps well to LUTs
// and avoids a priority chain.
assign alu_result = ({32{op_add | op_sub}} & add_sub_result)
                  | ({32{op_slt         }} & slt_result)
                  | ({32{op_sltu        }} & sltu_result)
                  | ({32{op_and         }} & and_result)
                  | ({32{op_nor         }} & nor_result)
                  | ({32{op_or          }} & or_result)
                  | ({32{op_xor         }} & xor_result)
                  | ({32{op_lui         }} & lui_result)
                  | ({32{op_sll         }} & sll_result)
                  | ({32{op_srl         }} & srl_result)
                  | ({32{op_sra         }} & sra_result);

endmodule
