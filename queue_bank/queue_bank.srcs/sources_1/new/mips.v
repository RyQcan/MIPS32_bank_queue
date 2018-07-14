module Top #(parameter WIDTH = 32, REGBITS = 5, MEMUNITS = 4096)(CLK,Reset,up,down,left,sw,AN,tub);
	input up,down,left, CLK,Reset;
    input [7:0] sw;

    output  [3:0] AN;
    output  [6:0] tub;

	wire    [2:0] MemWrite,MemRead; 
	wire    CS;
	wire    [WIDTH-1:0]  MemReadData;
	wire    [WIDTH-1:0]  IOReadData;
	wire	 [WIDTH-1:0]  ReadData;
	wire    [15:0] Addr;
	wire	 [WIDTH-1:0] WriteData;    
    assign   CS=Addr[14];  //I/O devices or Memory
    Mux2 Mem_or_IORead(MemReadData,IOReadData,CS,ReadData);
        
	MIPS #(WIDTH, REGBITS, MEMUNITS) Processor(CLK, Reset, Addr, MemWrite, MemRead, WriteData, ReadData);
	
	MemoryController #(WIDTH, MEMUNITS) Memory(CLK,CS, Addr, MemWrite, MemRead, WriteData, MemReadData);
	
	IO_Controller#(WIDTH) IO(CLK, CS, Addr, MemWrite, MemRead, WriteData, IOReadData,up,down,left,sw,AN,tub);

endmodule

module IO_Controller#(parameter WIDTH = 32)
						(CLK, CS, Addr, MemWrite, MemRead, WriteData, ReadData,up,down,left,sw,AN,tub);
	input		CLK,CS;
	input       [2:0] MemWrite, MemRead;
	input		[31:0] WriteData;
	input       [15:0] Addr;
	output reg 	[WIDTH-1:0] ReadData;
	input up,down,left;
	input [7:0] sw;

	output reg [3:0] AN;
	output reg [6:0] tub;

	reg temp[31:0];
	initial 
	begin
	   AN<=4'b1111;
	   tub<=7'b11111111;

	end
		
	wire [15:0] AlignedAddr;
	assign AlignedAddr 	  = Addr & 'hFFFC;
	
	//I/O devices address
	parameter up_addr = 16'h4000;
	parameter down_addr = 16'h4008;
    parameter left_addr = 16'h4010;
	parameter sw_addr = 16'h400c;

	parameter AN_addr = 16'h4014;
	parameter tub_addr = 16'h4018;


	always @(posedge CLK)
		if (MemWrite>0)
		  if (CS)
			case (AlignedAddr)
				AN_addr:AN[3:0]<=WriteData[3:0];
				tub_addr:tub[6:0]<=WriteData[6:0];
				
			endcase
			
	// read when MemRead
	always @(*) 
		if (MemRead>0) 
		  if (CS)
			case(AlignedAddr)
				up_addr: ReadData<={31'b0,up};
				down_addr:ReadData<={31'b0,down};
                left_addr:ReadData<={31'b0,left};
				sw_addr:ReadData<={24'b0,sw[7:0]};
				
		endcase
endmodule

module MemoryController #(parameter WIDTH = 32, MEMUNITS = 4096)
						 (CLK, CS, Addr, MemWrite, MemRead, WriteData, ReadData);
	
	input		CLK,CS;
	input       [2:0] MemWrite, MemRead;
	input		[WIDTH-1:0] WriteData;
	input       [15:0] Addr;
	output reg 	[WIDTH-1:0] ReadData;
	// [MEMUNITS * WIDTH (bit)] bytes RAM 
	reg			[WIDTH-1:0] RAM [MEMUNITS-1:0];
	// Align addr `automatically`, and calc offset for read/write
	// But should throw execution if not aligned.
    wire [15:0] AlignedAddr, OffsetHalfWord, OffsetByte;
    assign AlignedAddr       = Addr & 'hFFFC;
    assign OffsetHalfWord = (Addr - AlignedAddr) & 'hFFFE;
    assign OffsetByte     = Addr - AlignedAddr;
    // Memory Operations
        parameter MEMWORD        = 3'b001;    // word
        parameter MEMWORDLEFT    = 3'b010;    // word (left)
        parameter MEMWORDRIGHT    = 3'b011;    // word (right)
        parameter MEMHALFWORD    = 3'b100;    // half-word
        parameter MEMHALFWORDU    = 3'b101;    // half-word (unsigned)
        parameter MEMBYTE        = 3'b110;    // byte
        parameter MEMBYTEU        = 3'b111;    // byte (unsigned)
	// load instructions into memory
	initial
		$readmemh("bank_queue.dat", RAM);
		//$readmemh("test.dat", RAM);
	// write when MemWrite & positive edge triggered
	always @(posedge CLK)
	if (CS==0)
			case (MemWrite)
                MEMWORD:      RAM[AlignedAddr>>2][31:0] <= WriteData; 
                MEMWORDLEFT: case (OffsetByte)
                                3: RAM[AlignedAddr>>2][31:0]  <= WriteData;
                                2: RAM[AlignedAddr>>2][23:0]  <= WriteData[31:8];
                                1: RAM[AlignedAddr>>2][15:0]  <= WriteData[31:16];
                                0: RAM[AlignedAddr>>2][7:0]   <= WriteData[31:24];
                             endcase
                MEMWORDRIGHT:case (OffsetByte)
                                3: RAM[AlignedAddr>>2][31:24] <= WriteData[7:0];
                                2: RAM[AlignedAddr>>2][31:16] <= WriteData[15:0];
                                1: RAM[AlignedAddr>>2][31:8]  <= WriteData[23:0];
                                0: RAM[AlignedAddr>>2][31:0]  <= WriteData;
                             endcase
                MEMHALFWORD:  // OffsetHalfWord is `automatically` aligned to half-word here, 
                              // but should throw execution if not.
                             case (OffsetHalfWord)
                                    2: RAM[AlignedAddr>>2][31:16] <= WriteData[15:0];
                                    0: RAM[AlignedAddr>>2][15:0]  <= WriteData[15:0];
                             endcase
                MEMBYTE:     case (OffsetByte)
                                    3: RAM[AlignedAddr>>2][31:24] <= WriteData[7:0];
                                    2: RAM[AlignedAddr>>2][23:16] <= WriteData[7:0];
                                    1: RAM[AlignedAddr>>2][15:8]  <= WriteData[7:0];
                                    0: RAM[AlignedAddr>>2][7:0]   <= WriteData[7:0];
                             endcase
            endcase
	// read when MemRead
	always @(*)
		if (CS==0)	
			case (MemRead)
                    MEMWORD:       ReadData <= RAM[AlignedAddr>>2][31:0];
                    MEMWORDLEFT:  begin
                                    ReadData <= WriteData;
                                    case (OffsetByte)
                                        3: ReadData        <= RAM[AlignedAddr>>2][31:0]; 
                                        2: ReadData[31:8]  <= RAM[AlignedAddr>>2][23:0];
                                        1: ReadData[31:16] <= RAM[AlignedAddr>>2][15:0];
                                        0: ReadData[31:24] <= RAM[AlignedAddr>>2][7:0];
                                    endcase
                                  end
                    MEMWORDRIGHT: begin
                                    ReadData <= WriteData;
                                    case (OffsetByte)
                                        3: ReadData[7:0]   <= RAM[AlignedAddr>>2][31:24];
                                        2: ReadData[15:0]  <= RAM[AlignedAddr>>2][31:16];
                                        1: ReadData[23:0]  <= RAM[AlignedAddr>>2][31:8];
                                        0: ReadData           <= RAM[AlignedAddr>>2][31:0];
                                    endcase
                                  end
                    MEMHALFWORD:  case (OffsetHalfWord)
                                        2: ReadData <= {{16{RAM[AlignedAddr>>2][31]}}, 
                                                        RAM[AlignedAddr>>2][31:16]};     // sign-extend
                                        0: ReadData <= {{16{RAM[AlignedAddr>>2][15]}}, 
                                                        RAM[AlignedAddr>>2][15:0]};        // sign-extend
                                  endcase
                    MEMHALFWORDU: case (OffsetHalfWord)
                                        2: ReadData <= {16'b0, 
                                                        RAM[AlignedAddr>>2][31:16]};    // zero-extend
                                        0: ReadData <= {16'b0, 
                                                        RAM[AlignedAddr>>2][15:0]};        // zero-extend
                                  endcase
                    MEMBYTE:      case (OffsetByte)
                                        3: ReadData <= {{24{RAM[AlignedAddr>>2][31]}}, 
                                                        RAM[AlignedAddr>>2][31:24]};     // sign-extend
                                        2: ReadData <= {{24{RAM[AlignedAddr>>2][23]}}, 
                                                        RAM[AlignedAddr>>2][23:16]};     // sign-extend
                                        1: ReadData <= {{24{RAM[AlignedAddr>>2][15]}}, 
                                                        RAM[AlignedAddr>>2][15:8]};     // sign-extend
                                        0: ReadData <= {{24{RAM[AlignedAddr>>2][7]}}, 
                                                        RAM[AlignedAddr>>2][7:0]};         // sign-extend
                                  endcase
                    MEMBYTEU:      case (OffsetByte)
                                        3: ReadData <= {24'b0, 
                                                        RAM[AlignedAddr>>2][31:24]};     // zero-extend
                                        2: ReadData <= {24'b0, 
                                                        RAM[AlignedAddr>>2][23:16]};     // zero-extend
                                        1: ReadData <= {24'b0, 
                                                        RAM[AlignedAddr>>2][15:8]};     // zero-extend
                                        0: ReadData <= {24'b0, 
                                                        RAM[AlignedAddr>>2][7:0]};         // zero-extend
                                  endcase
                endcase
endmodule

module MIPS #(parameter WIDTH = 32, REGBITS = 5, MEMUNITS = 4096)
			 (CLK, Reset, Addr, MemWrite, MemRead, WriteData, ReadData);
		
	input				CLK, Reset;
	input	[WIDTH-1:0] ReadData;
	output  [2:0] MemWrite, MemRead;
	output	[WIDTH-1:0] WriteData;
	output  [15:0] Addr;
	// PC/IR
	wire [31:0] Instr;
	wire  		InstrOrData, PCEn, IREn, PCSrc;
	
	// Register
	wire		RegWrite, MemToReg;
	wire [1:0]  RegDst;
	
	// MainController
	wire [3:0]  CurrentState;
	wire [2:0]  ALUSignCond; // Condition for EQ/LT/GT/NE/GE/LE
	
	// ALU
	wire		ALUZero, ALUSign, ALUOverflow; // ALU Flag
	wire        ALUCompare; // Compare result for specific condition
	wire [1:0] 	ALUSrcA;
	wire [2:0]  ALUSrcB;
	wire [4:0]  ALUControl;
	MainController  Control(CLK, Reset,
							Instr[31:26], Instr[5:0], Instr[20:16],
							ALUZero, ALUSign, ALUOverflow, ALUCompare,
							InstrOrData, IREn, PCSrc, PCEn,
							MemWrite, MemRead,
							RegWrite, RegDst, MemToReg,
							ALUSrcA, ALUSrcB, ALUSignCond, CurrentState);
	ALUDecoder  	Decode(CurrentState, Instr[31:26], Instr[5:0], ALUControl);
	DataPath 		#(WIDTH, REGBITS, MEMUNITS)
					Path(CLK, Reset, 
						 Addr, WriteData, ReadData,
						 Instr, InstrOrData, PCEn, IREn, PCSrc,
						 RegWrite, RegDst, MemToReg,
						 ALUZero, ALUSign, ALUOverflow, ALUCompare,
						 ALUSrcA, ALUSrcB, ALUControl, ALUSignCond);
	
endmodule

module MainController (CLK, Reset,
					   OpCode, Func, RegImmType,
					   ALUZero, ALUSign, ALUOverflow, ALUCompare,
					   InstrOrData, IREn, PCSrc, PCEn,
					   MemWrite, MemRead, 
					   RegWrite, RegDst, MemToReg,
					   ALUSrcA, ALUSrcB, ALUSignCond, CurrentState);
		
	input 				CLK, Reset;
	input		  [5:0] OpCode, Func;
	input		  [4:0] RegImmType;
	input  		    	ALUZero, ALUSign, ALUOverflow;
	input				ALUCompare;
	
	output reg			InstrOrData, IREn, PCSrc;
	output				PCEn, RegWrite;
	output reg	  [2:0] MemWrite, MemRead;
	output reg			MemToReg;
	output reg	  [1:0] RegDst, ALUSrcA;
	output reg    [2:0] ALUSrcB, ALUSignCond;
	output    	  [3:0] CurrentState;
	
	// Period 1
	parameter FETCH     	=  4'b0001;
	// Period 2
	parameter DECODE    	=  4'b0010;
	// Period 3
	parameter JUMPLINK		=  4'b0011;
	parameter MEMADDR		=  4'b0100;
	parameter JUMP			=  4'b0101;
	parameter EXECUTE		=  4'b0110;
	// Period 4
	parameter MEMWRITE		=  4'b0111;
	parameter MEMREAD		=  4'b1000;
	parameter ALUTOREG 	 	=  4'b1001;
	parameter BRLINK		=  4'b1010;
	parameter BRANCH		=  4'b1011;
	// Period 5
	parameter MEMTOREG  	=  4'b1100;
	
	// R-Type Instructions
	parameter SPECIAL		= 6'b000000;
	// SPECIAL Funcs
	parameter SLL			= 6'b000000;
	parameter SRL			= 6'b000010;
	parameter SRA			= 6'b000011;
	parameter SLLV			= 6'b000100;
	parameter SRLV			= 6'b000110;
	parameter SRAV			= 6'b000111;
	parameter JR			= 6'b001000;
	parameter JALR			= 6'b001001;
	parameter MOVZ			= 6'b001010;
	parameter MOVN			= 6'b001011;
	parameter ADD			= 6'b100000;
	parameter ADDU			= 6'b100001;
	parameter SUB			= 6'b100010;
	parameter SUBU			= 6'b100011;
	parameter AND			= 6'b100100;
	parameter OR			= 6'b100101;
	parameter XOR			= 6'b100110;
	parameter NOR			= 6'b100111;
	parameter SLT			= 6'b101010;
	parameter SLTU			= 6'b101011;

	
	// J-Type Instructions
	parameter J				= 6'b000010;
	parameter JAL			= 6'b000011;
	
	// I-Type Instructions
	parameter REGIMM		= 6'b000001;
	// REGIMM Types Start
	parameter BLTZ			= 5'b00000; 
	parameter BGEZ			= 5'b00001;
	parameter BLTZAL		= 5'b10000;
	parameter BGEZAL		= 5'b10001;
	// REGIMM Types End
	parameter BEQ			= 6'b000100;
	parameter BNE			= 6'b000101;
	parameter BLEZ			= 6'b000110;
	parameter BGTZ			= 6'b000111;
	parameter ADDI			= 6'b001000;
	parameter ADDIU			= 6'b001001;
	parameter SLTI			= 6'b001010;
	parameter SLTIU			= 6'b001011;
	parameter ANDI			= 6'b001100;
	parameter XORI			= 6'b001110;
	parameter ORI			= 6'b001101;
	parameter LUI			= 6'b001111;
	parameter LB			= 6'b100000;
    parameter LH            = 6'b100001;
    parameter LBU            = 6'b100100;
    parameter LHU            = 6'b100101;
    parameter LWL            = 6'b100010;
    parameter LW            = 6'b100011;
    parameter LWR            = 6'b100110;
    parameter SB            = 6'b101000;
    parameter SH            = 6'b101001;
    parameter SWL            = 6'b101010;
    parameter SW            = 6'b101011;
    parameter SWR            = 6'b101110;
    
    // Memory Operations
    parameter MEMWORD        = 3'b001;    // word
    parameter MEMWORDLEFT    = 3'b010;    // word (left)
    parameter MEMWORDRIGHT    = 3'b011;    // word (right)
    parameter MEMHALFWORD    = 3'b100;    // half-word
    parameter MEMHALFWORDU    = 3'b101;    // half-word (unsigned)
    parameter MEMBYTE        = 3'b110;    // byte
    parameter MEMBYTEU        = 3'b111;    // byte (unsigned)
	reg [3:0] 	State, NextState;
	reg			PCWrite, Branch;
	reg			RegSet, RegCondSet;
	
	// state register
	assign CurrentState = State;
	always @(posedge CLK)
		if (Reset) State <= FETCH;
		else State <= NextState;
		
	// next state logic
	always @(*)
		case (State)
			// Period 1
			FETCH: 		NextState <= DECODE;
			// Period 2
			DECODE: 	case (OpCode)
							// R-Type
							SPECIAL: NextState <= EXECUTE;
							// J-Type
							J: 		 NextState <= JUMP;    // J
							JAL:	 NextState <= JUMPLINK;// JAL
							// I-Type
							REGIMM:  NextState <= EXECUTE; // BGEZ, BGEZAL, BLTZ, BLTZAL
							BEQ: 	 NextState <= EXECUTE; // BEQ
							BNE: 	 NextState <= EXECUTE; // BNE
							BLEZ: 	 NextState <= EXECUTE; // BLEZ
							BGTZ: 	 NextState <= EXECUTE; // BGTZ
							ADDI: 	 NextState <= EXECUTE; // ADDI
							ADDIU: 	 NextState <= EXECUTE; // ADDIU
							SLTI: 	 NextState <= EXECUTE; // SLTI
							SLTIU: 	 NextState <= EXECUTE; // SLTIU
							ANDI: 	 NextState <= EXECUTE; // ANDI
							XORI: 	 NextState <= EXECUTE; // XORI
							ORI:   	 NextState <= EXECUTE; // ORI
							LUI: 	 NextState <= EXECUTE; // LUI
							LB:		 NextState <= MEMADDR; // LB
                            LH:         NextState <= MEMADDR; // LH
                            LBU:     NextState <= MEMADDR; // LBU
                            LHU:     NextState <= MEMADDR; // LHU
                            LWL:     NextState <= MEMADDR; // LWL
                            LW:      NextState <= MEMADDR; // LW
                            LWR:     NextState <= MEMADDR; // LWR
                            SB:         NextState <= MEMADDR; // SB
                            SH:         NextState <= MEMADDR; // SH
                            SWL:     NextState <= MEMADDR; // SWL
                            SW:         NextState <= MEMADDR; // SW
                            SWR:     NextState <= MEMADDR; // SWR
							default: NextState <= FETCH;   // should never happen
						endcase
			// Period 3
			JUMPLINK: 	NextState <= JUMP;
			MEMADDR: 	case (OpCode)
							LB: 	 NextState <= MEMREAD;  // LB
                            LH:         NextState <= MEMREAD;  // LH
                            LBU:     NextState <= MEMREAD;  // LBU
                            LHU:     NextState <= MEMREAD;  // LHU
                            LWL:      NextState <= MEMREAD;  // LWL
                            LW:      NextState <= MEMREAD;  // LW
                            LWR:     NextState <= MEMREAD;  // LWR
                            SB:         NextState <= MEMWRITE; // SB
                            SH:         NextState <= MEMWRITE; // SH
                            SWL:     NextState <= MEMWRITE; // SWL
                            SW:      NextState <= MEMWRITE; // SW
                            SWR:     NextState <= MEMWRITE; // SWR
							default: NextState <= FETCH;	// should never happen
						 endcase
			JUMP: 		NextState <= FETCH;
			EXECUTE: 	case (OpCode)
							// R-Type
							SPECIAL: case (Func)
										JR: 	 NextState <= JUMP;  	  // JR
										JALR:	 NextState <= JUMPLINK;   // JALR
										default: NextState <= ALUTOREG;	  // other R-Type instr
									 endcase
							// I-Type
							REGIMM:  case (RegImmType)
										BLTZ: 	 NextState <= BRANCH; // BLTZ
										BGEZ: 	 NextState <= BRANCH; // BGEZ
										BLTZAL:  NextState <= BRLINK; // BLTZAL
										BGEZAL:  NextState <= BRLINK; // BGEZAL
										default: NextState <= FETCH;  // should never happen
									 endcase
							BEQ: 	 NextState <= BRANCH;   // BEQ
							BNE: 	 NextState <= BRANCH;	// BNE
							BLEZ: 	 NextState <= BRANCH;	// BLEZ
							BGTZ: 	 NextState <= BRANCH;   // BGTZ
							ADDI: 	 NextState <= ALUTOREG; // ADDI
							ADDIU: 	 NextState <= ALUTOREG; // ADDIU
							SLTI: 	 NextState <= ALUTOREG; // SLTI
							SLTIU: 	 NextState <= ALUTOREG; // SLTIU
							ANDI: 	 NextState <= ALUTOREG; // ANDI
							XORI: 	 NextState <= ALUTOREG; // XORI
							ORI: 	 NextState <= ALUTOREG; // ORI
							LUI: 	 NextState <= ALUTOREG; // LUI
							default: NextState <= FETCH;	// should never happen
						endcase
			// Period 4
			MEMWRITE: 	NextState <= FETCH;
			MEMREAD: 	NextState <= MEMTOREG;
			ALUTOREG: 	NextState <= FETCH;
			BRLINK:		NextState <= BRANCH;
			BRANCH: 	NextState <= FETCH;
			// Period 5
			MEMTOREG: 	NextState <= FETCH;
			// Should never happen
			default: 	NextState <= FETCH;
		endcase
		
	// current state execution
	always @(*)
		begin
			PCWrite <= 0; Branch <= 0;
			InstrOrData <= 0; IREn <= 0; PCSrc <= 0;
			MemWrite <= 3'b0; MemRead <= 3'b0;
			RegSet <= 0; RegCondSet <= 0; RegDst <= 2'b0; MemToReg <= 0;
			ALUSrcA <= 2'b0; ALUSrcB <= 3'b0; ALUSignCond <= 3'b111;
			
			case (State)
				// Period 1
				FETCH:
					begin
						MemRead <= MEMWORD; // an instruction is 4-byte (a word)
						IREn <= 1;
						PCWrite <= 1;
						ALUSrcB <= 3'b001;
					end
				// Period 2
				DECODE: case (OpCode)
							J:   ALUSrcB <= 3'b100; // calc jump address, when J, we can directly JUMP when Period 3.
							JAL: ALUSrcB <= 3'b111; // return address should have be PC+4, but we've done it at FETCH,
													// so just add 0 to current PC, and when JAL, 
											// we can directly JUMPLINK when Period 3
						endcase 
				// Period 3
				JUMPLINK:
					begin
						RegSet <= 1;
						case (OpCode)
							// Func should be JALR
							SPECIAL:begin
										RegDst <= 2'b01;   // RD
										ALUSrcA <= 2'b01;  // jump address at RS, no need to set ALUSrcB,
														   // since RT = 0 in JALR
									end
							JAL:	begin
										RegDst <= 2'b10;   // $31($ra)
										ALUSrcB <= 3'b100; // calc jump address
									end
						endcase
					end
				MEMADDR:
					begin
						ALUSrcA <= 2'b01;
						ALUSrcB <= 3'b010; // calc memory address
					end
				JUMP:
					begin
						PCWrite <= 1;
						PCSrc <= 1;
					end
				EXECUTE:
					begin
						case (OpCode)
							// R-Type
							SPECIAL: case (Func)
										SLL:	 ALUSrcA <= 2'b10;  // instr[10:6] - SA
										SRA:	 ALUSrcA <= 2'b10;  // instr[10:6] - SA
										SRL:	 ALUSrcA <= 2'b10;  // instr[10:6] - SA
										JALR:	 ALUSrcB <= 3'b111; // return address should have be PC+4, but we've done it at FETCH,
																	// so just add 0 to current PC
										MOVZ:	 begin				// RS+0
													ALUSrcA <= 2'b01; 
													ALUSrcB <= 3'b111; 
												 end
										MOVN:	 begin				// RS+0
													ALUSrcA <= 2'b01; 
													ALUSrcB <= 3'b111; 
												 end
										default: ALUSrcA <= 2'b01;  // instr[25:21] - RS
									 endcase
							// I-Type
							REGIMM:  case (RegImmType)
										BLTZ: 	ALUSrcB <= 3'b011; // BLTZ, calc jump address
										BGEZ:	ALUSrcB <= 3'b011; // BGEZ, calc jump address
										BLTZAL: ALUSrcB <= 3'b111; // return address should have be PC+4, but we've done it at FETCH,
																   // so just add 0 to current PC
										BGEZAL: ALUSrcB <= 3'b111; // return address should have be PC+4, but we've done it at FETCH,
																   // so just add 0 to current PC
									 endcase
							BEQ: 	 ALUSrcB <= 3'b011; // BEQ, calc jump address
							BNE: 	 ALUSrcB <= 3'b011; // BNE, calc jump address
							BLEZ: 	 ALUSrcB <= 3'b011; // BLEZ, calc jump address
							BGTZ: 	 ALUSrcB <= 3'b011; // BGTZ, calc jump address
							ADDI: 	 begin ALUSrcA <= 2'b01; ALUSrcB <= 3'b010; end // ADDI
							ADDIU: 	 begin ALUSrcA <= 2'b01; ALUSrcB <= 3'b010; end // ADDIU
							SLTI: 	 begin ALUSrcA <= 2'b01; ALUSrcB <= 3'b010; end // SLTI
							SLTIU: 	 begin ALUSrcA <= 2'b01; ALUSrcB <= 3'b010; end // SLTIU
							ANDI:	 begin ALUSrcA <= 2'b01; ALUSrcB <= 3'b101; end // ANDI
							XORI:	 begin ALUSrcA <= 2'b01; ALUSrcB <= 3'b101; end // XORI
							ORI: 	 begin ALUSrcA <= 2'b01; ALUSrcB <= 3'b101; end // ORI
							LUI: 	 begin ALUSrcA <= 2'b01; ALUSrcB <= 3'b010; end // LUI
						endcase
					end
				// Period 4
				MEMWRITE:
					begin
						InstrOrData <= 1;
                        case (OpCode)
                                SB:  MemWrite <= MEMBYTE;      // SB
                                SH:  MemWrite <= MEMHALFWORD; // SH
                                SWL: MemWrite <= MEMWORDLEFT; // SWL
                                SW:  MemWrite <= MEMWORD;       // SW
                                SWR: MemWrite <= MEMWORDRIGHT;// SWR
                        endcase
					end
				MEMREAD:
					begin
						InstrOrData <= 1;
						case (OpCode)
                              LB:  MemRead <= MEMBYTE;      // LB
                              LH:  MemRead <= MEMHALFWORD;  // LH
                              LBU: MemRead <= MEMBYTEU;      // LBU
                              LHU: MemRead <= MEMHALFWORDU; // LHU
                              LWL: MemRead <= MEMWORDLEFT;  // LWL
                              LW:  MemRead <= MEMWORD;       // LW
                              LWR: MemRead <= MEMWORDRIGHT; // LWR
                        endcase
					end
				ALUTOREG:
					begin
						case (OpCode)
							// R-Type Instructions
							SPECIAL:  case (Func)  
										MOVZ:    begin
													ALUSrcA <= 2'b11; // 0+RT
													RegCondSet <= 1; ALUSignCond <= 3'b010; 
													RegDst <= 2'b01; 
												 end
										MOVN:    begin 
													ALUSrcA <= 2'b11; // 0+RT
													RegCondSet <= 1; ALUSignCond <= 3'b100; 
													RegDst <= 2'b01; 
												 end
										default: begin RegSet <= 1; RegDst <= 2'b01; end
									  endcase
							// I-Type Instructions
							default:  RegSet <= 1;
						endcase
					end
				BRLINK:
					begin
						RegSet <= 1;
						RegDst <= 2'b10; 	// $31($ra)
						ALUSrcB <= 3'b011;  // calc jump address
					end
				BRANCH:
					begin
						Branch <= 1; 
						PCSrc <= 1;
						ALUSrcA <= 2'b01;			
						case (OpCode)
							REGIMM: begin
										ALUSrcB <= 3'b111;				   // REGIMM's instr[20:16] is type
										case (RegImmType)
											BLTZ:   ALUSignCond <= 3'b001; // BLTZ
											BGEZ:   ALUSignCond <= 3'b101; // BGEZ
											BLTZAL: ALUSignCond <= 3'b001; // BLTZAL
											BGEZAL: ALUSignCond <= 3'b101; // BGEZAL
										endcase
									end
							BEQ:  ALUSignCond <= 3'b010;         // BEQ
							BNE:  ALUSignCond <= 3'b100;         // BNE
							BLEZ: ALUSignCond <= 3'b110;         // BLEZ
							BGTZ: ALUSignCond <= 3'b000;         // BGTZ
						endcase
					end
				// Period 5
				MEMTOREG:
					begin
						RegSet <= 1;
						MemToReg <= 1;
					end
			endcase
		end
	
	assign PCEn     = PCWrite | (Branch & ALUCompare);
	assign RegWrite = RegSet  | (RegCondSet & ALUCompare);
endmodule

module ALUDecoder (CurrentState, OpCode, Func, ALUControl);

	input       [3:0] CurrentState;
	input 		[5:0] OpCode;
	input 		[5:0] Func;
	output reg  [4:0] ALUControl;

	// Period 1
	parameter FETCH     	=  4'b0001;
	// Period 2
	parameter DECODE    	=  4'b0010;
	// Period 3
	parameter JUMPLINK		=  4'b0011;
	parameter MEMADDR		=  4'b0100;
	parameter JUMP			=  4'b0101;
	parameter EXECUTE		=  4'b0110;
	// Period 4
	parameter MEMWRITE		=  4'b0111;
	parameter MEMREAD		=  4'b1000;
	parameter ALUTOREG 	 	=  4'b1001;
	parameter BRLINK		=  4'b1010;
	parameter BRANCH		=  4'b1011;
	// Period 5
	parameter MEMTOREG  	=  4'b1100;
	
	// R-Type Instructions
	parameter SPECIAL		= 6'b000000;
	// SPECIAL Funcs
	parameter SLL			= 6'b000000;
	parameter SRL			= 6'b000010;
	parameter SRA			= 6'b000011;
	parameter SLLV			= 6'b000100;
	parameter SRLV			= 6'b000110;
	parameter SRAV			= 6'b000111;
	parameter JR			= 6'b001000;
	parameter JALR			= 6'b001001;
	parameter MOVZ			= 6'b001010;
	parameter MOVN			= 6'b001011;
	parameter ADD			= 6'b100000;
	parameter ADDU			= 6'b100001;
	parameter SUB			= 6'b100010;
	parameter SUBU			= 6'b100011;
	parameter AND			= 6'b100100;
	parameter OR			= 6'b100101;
	parameter XOR			= 6'b100110;
	parameter NOR			= 6'b100111;
	parameter SLT			= 6'b101010;
	parameter SLTU			= 6'b101011;

	
	// J-Type Instructions
	parameter J				= 6'b000010;
	parameter JAL			= 6'b000011;
	
	// I-Type Instructions
	parameter REGIMM		= 6'b000001;
	// REGIMM Types Start
	parameter BLTZ			= 5'b00000; 
	parameter BGEZ			= 5'b00001;
	parameter BLTZAL		= 5'b10000;
	parameter BGEZAL		= 5'b10001;
	// REGIMM Types End
	parameter BEQ			= 6'b000100;
	parameter BNE			= 6'b000101;
	parameter BLEZ			= 6'b000110;
	parameter BGTZ			= 6'b000111;
	parameter ADDI			= 6'b001000;
	parameter ADDIU			= 6'b001001;
	parameter SLTI			= 6'b001010;
	parameter SLTIU			= 6'b001011;
	parameter ANDI			= 6'b001100;
	parameter XORI			= 6'b001110;
	parameter ORI			= 6'b001101;
	parameter LUI			= 6'b001111;
	parameter LB			= 6'b100000;
    parameter LH            = 6'b100001;
    parameter LBU            = 6'b100100;
    parameter LHU            = 6'b100101;
    parameter LWL            = 6'b100010;
    parameter LW            = 6'b100011;
    parameter LWR            = 6'b100110;
    parameter SB            = 6'b101000;
    parameter SH            = 6'b101001;
    parameter SWL            = 6'b101010;
    parameter SW            = 6'b101011;
    parameter SWR            = 6'b101110;
	
	// ALU Control Code
	parameter ALUADD		= 5'b00000;
	parameter ALUADDU		= 5'b00001;
	parameter ALUSUB		= 5'b00010;
	parameter ALUSUBU		= 5'b00011;
	parameter ALUAND		= 5'b00100;
	parameter ALUOR			= 5'b00101;
	parameter ALUNOR		= 5'b00110;
	parameter ALUXOR		= 5'b00111;
	parameter ALULT			= 5'b01000;
	parameter ALULTU		= 5'b01001;
	parameter ALUSLL		= 5'b01010;
	parameter ALUSRL		= 5'b01011;
	parameter ALUSRA		= 5'b01100;
	parameter ALULUI		= 5'b01101;
	parameter ALUJ			= 5'b01110;
	parameter ALUCLO		= 5'b01111;
	parameter ALUCLZ		= 5'b10000;
	parameter ALUNOP		= 5'b11111;

	always @(*)
		case (CurrentState)
			FETCH:   ALUControl <= ALUADDU;  		// nextpc = pc + 4
			DECODE:  case (OpCode)
						J: 	 ALUControl <= ALUJ;    // calc jump address
						JAL: ALUControl <= ALUADDU; // return address should have be PC+4, but we've done it at FETCH,
							// so just add 0 to current PC.
					 endcase 
			MEMADDR: ALUControl <= ALUADDU;  		// base+offset
			BRANCH:  ALUControl <= ALUSUB;   		// subtract two operands, for all branch instr
			default: case (OpCode)
						// R-Type
						SPECIAL:  case (Func) 
									SLL: 	 ALUControl <= ALUSLL; 	// SLL
									SRL: 	 ALUControl <= ALUSRL; 	// SRL
									SRA: 	 ALUControl <= ALUSRA; 	// SRA
									SLLV: 	 ALUControl <= ALUSLL; 	// SLLV
									SRLV: 	 ALUControl <= ALUSRL; 	// SRLV
									SRAV: 	 ALUControl <= ALUSRA; 	// SRAV
									JR: 	 ALUControl <= ALUADDU; // JR
									JALR:	 ALUControl <= ALUADDU; // JALR
									MOVZ:	 ALUControl <= ALUADDU; // MOVZ
									MOVN:	 ALUControl <= ALUADDU; // MOVN
									ADD: 	 ALUControl <= ALUADD; 	// ADD
									ADDU: 	 ALUControl <= ALUADDU; // ADDU
									SUB: 	 ALUControl <= ALUSUB; 	// SUB
									SUBU: 	 ALUControl <= ALUSUBU; // SUBU
									AND: 	 ALUControl <= ALUAND; 	// AND
									OR: 	 ALUControl <= ALUOR; 	// OR
									XOR: 	 ALUControl <= ALUXOR; 	// XOR
									NOR: 	 ALUControl <= ALUNOR; 	// NOR
									SLT: 	 ALUControl <= ALULT; 	// SLT
									SLTU: 	 ALUControl <= ALULTU; 	// SLTU
									default: ALUControl <= ALUNOP; 	// should never happen, set to NOP
								 endcase

						// J-Type
						JAL:	 ALUControl <= ALUJ; 	// JAL, calc jump address (ALUJ) when JUMPLINK
						// I-Type
						REGIMM:  ALUControl <= ALUADDU; // BGEZ, BLTZ, calc jump address (ADDU) when EXECUTE
														// BLTZAL, BGEZAL, calc return address (ADDU) when EXECUTE
														// BLTZAL, BGEZAL, calc jump address (ADDU) when BRLINK
						BEQ: 	 ALUControl <= ALUADDU; // BEQ, calc jump address (ADDU) when EXECUTE
						BNE:	 ALUControl <= ALUADDU; // BNE, calc jump address (ADDU) when EXECUTE
						BLEZ: 	 ALUControl <= ALUADDU; // BLEZ, calc jump address (ADDU) when EXECUTE
						BGTZ: 	 ALUControl <= ALUADDU; // BGTZ, calc jump address (ADDU) when EXECUTE
						ADDI: 	 ALUControl <= ALUADD; 	// ADDI
						ADDIU: 	 ALUControl <= ALUADDU; // ADDIU
						SLTI: 	 ALUControl <= ALULT; 	// SLTI
						SLTIU: 	 ALUControl <= ALULTU;	// SLTIU
						ANDI: 	 ALUControl <= ALUAND; 	// ANDI
						XORI:	 ALUControl <= ALUXOR; 	// XORI
						ORI: 	 ALUControl <= ALUOR; 	// ORI
						LUI: 	 ALUControl <= ALULUI; 	// LUI
						LB: 	 ALUControl <= ALUADDU; // LB, calc address (ADDU) when MEMADDR
                        LH:      ALUControl <= ALUADDU; // LH, calc address (ADDU) when MEMADDR
                        LBU:      ALUControl <= ALUADDU; // LBU, calc address (ADDU) when MEMADDR
                        LHU:      ALUControl <= ALUADDU; // LHU, calc address (ADDU) when MEMADDR
                        LWL:     ALUControl <= ALUADDU; // LWL, calc address (ADDU) when MEMADDR
                        LW:      ALUControl <= ALUADDU; // LW, calc address (ADDU) when MEMADDR
                        LWR:     ALUControl <= ALUADDU; // LWR, calc address (ADDU) when MEMADDR
                        SB:      ALUControl <= ALUADDU; // SB, calc address (ADDU) when MEMADDR
                        SH:      ALUControl <= ALUADDU; // SH, calc address (ADDU) when MEMADDR
                        SWL:     ALUControl <= ALUADDU; // SWL, calc address (ADDU) when MEMADDR
                        SW:      ALUControl <= ALUADDU; // SW, calc address (ADDU) when MEMADDR
                        SWR:     ALUControl <= ALUADDU; // SWR, calc address (ADDU) when MEMADDR
						default: ALUControl <= ALUNOP; 	// should never happen, set to NOP
					endcase
		endcase
		
endmodule

module DataPath #(parameter WIDTH = 32, REGBITS = 5, MEMUNITS = 4096)
				 (CLK, Reset, 
				 Addr, WriteData, ReadData,
				 Instr, InstrOrData, PCEn, IREn, PCSrc,
				 RegWrite, RegDst, MemToReg,
				 ALUZero, ALUSign, ALUOverflow, ALUCompare,
				 ALUSrcA, ALUSrcB, ALUControl, ALUSignCond);
				 
	input 				CLK, Reset;
	input	[WIDTH-1:0] ReadData;
	input				InstrOrData, PCEn, IREn, PCSrc;
	input				RegWrite, MemToReg;
	input		  [1:0] RegDst, ALUSrcA;
	input 		  [2:0] ALUSrcB, ALUSignCond;
	input		  [4:0] ALUControl;
	output  [WIDTH-1:0] WriteData;
	wire    [WIDTH-1:0] Addrtemp;
	output  [15:0] Addr;
	output       [31:0] Instr;
	output  		    ALUZero, ALUSign, ALUOverflow;
	output	 			ALUCompare;
	wire   [WIDTH-1:0]  PC, NextPC;
	wire   [WIDTH-1:0]  Data;
	wire [REGBITS-1:0]  A1, A2, A3;
	wire   [WIDTH-1:0] 	RD1, RD2, WD3, RS, RT;
	wire   [WIDTH-1:0]  ZeroExtSA, SignExtImm, ZeroExtIndex, ZeroExtImm;
	wire   [WIDTH-1:0]  SrcA, SrcB, ALUResult, ALUOut;
	
	// Imm Extend
	assign ZeroExtSA = {27'b0, Instr[10:6]}; 			// Zero Extend SA
	assign SignExtImm = {{16{Instr[15]}}, Instr[15:0]}; // Sign Extend Imm
	assign ZeroExtIndex = {4'b0, Instr[25:0], 2'b0}; 	// InstrIndex << 2, then Zero Extend 
	assign ZeroExtImm = {16'b0, Instr[15:0]}; 			// Zero Extend Imm
	
	// Memory
	assign WriteData = RT;
	assign Addr=Addrtemp[15:0];
	Mux2	  	     #(WIDTH) MuxAddr(PC, ALUOut, InstrOrData, Addrtemp);

	// Register Address
	assign A1 = Instr[25:21];
	assign A2 = Instr[20:16];
	
	// Registers
	FlopEn 	  	 #(WIDTH) 					 IRReg(CLK, IREn, ReadData, Instr);
	Flop		 #(WIDTH)					 DataReg(CLK, ReadData, Data);
	FlopEnReset  #(WIDTH) 					 PCReg(CLK, PCEn, Reset, NextPC, PC);
	RegisterFile #(WIDTH, REGBITS, MEMUNITS) RegFile(CLK, Reset, A1, A2, A3, RD1, RD2, RegWrite, WD3);
	Flop         #(WIDTH) 					 RSReg(CLK, RD1, RS);
	Flop         #(WIDTH)					 RTReg(CLK, RD2, RT);
	Mux4 	     #(REGBITS) 				 MuxA3(Instr[20:16], Instr[15:11], 5'b11111, 5'bx, RegDst, A3);
	Mux2         #(WIDTH) 					 MuxWD3(ALUOut, Data, MemToReg, WD3);
	
	// ALU
	ALU	      		#(WIDTH) ALUnit(SrcA, SrcB, ALUControl, ALUResult);
	ZeroDetect 		#(WIDTH) ZeroFlag(ALUResult, ALUZero);
	SignDetect 		#(WIDTH) SignFlag(ALUResult, ALUSign);
	OverflowDetect  #(WIDTH) OverflowFlag(ALUResult, SrcA, SrcB, ALUControl, ALUOverflow);
	SignedCompare            CompareDecode(ALUZero, ALUSign, ALUOverflow, ALUSignCond, ALUCompare);
	Flop      		#(WIDTH) ALUResultReg(CLK, ALUResult, ALUOut);
	Mux4      		#(WIDTH) MuxALUSrcA(PC, RS, ZeroExtSA, 32'b0, ALUSrcA, SrcA);
	Mux8	  		#(WIDTH) MuxALUSrcB(RT, 32'b100, SignExtImm, SignExtImm << 2, 
										ZeroExtIndex, ZeroExtImm, 32'bx, 32'b0, 
										ALUSrcB, SrcB);
										
	// Next PC
	Mux2		#(WIDTH) MuxNextPC(ALUResult, ALUOut, PCSrc, NextPC);
	
endmodule

module ALU #(parameter WIDTH = 32)
			(A, B, ALUControl, Result);
			
	input 		[WIDTH-1:0] A, B;
	input 			  [4:0] ALUControl;
	output reg  [WIDTH-1:0] Result;
	
	// ALU Control Code
	parameter ALUADD		= 5'b00000;
	parameter ALUADDU		= 5'b00001;
	parameter ALUSUB		= 5'b00010;
	parameter ALUSUBU		= 5'b00011;
	parameter ALUAND		= 5'b00100;
	parameter ALUOR			= 5'b00101;
	parameter ALUNOR		= 5'b00110;
	parameter ALUXOR		= 5'b00111;
	parameter ALULT			= 5'b01000;
	parameter ALULTU		= 5'b01001;
	parameter ALUSLL		= 5'b01010;
	parameter ALUSRL		= 5'b01011;
	parameter ALUSRA		= 5'b01100;
	parameter ALULUI		= 5'b01101;
	parameter ALUJ			= 5'b01110;
	parameter ALUCLO		= 5'b01111;
	parameter ALUCLZ		= 5'b10000;
	parameter ALUNOP		= 5'b11111;
	
	always @(*)
		case (ALUControl)
			ALUADD:  Result <= A + B; 						 			// Add
			ALUADDU: Result <= A + B; 					 	 			// AddU
			ALUSUB:  Result <= A - B; 						 			// Sub
			ALUSUBU: Result <= A - B; 						 			// SubU
			ALUAND:  Result <= A & B; 						 			// And
			ALUOR: 	 Result <= A | B; 									// Or
			ALUNOR:  Result <= ~(A | B); 					 			// Nor
			ALUXOR:  Result <= A ^ B; 						 			// Xor
			ALULT: 	 Result <= ($signed(A) < $signed(B)) ? 1 : 0; 		// LT
			ALULTU:  Result <= ($unsigned(A) < $unsigned(B)) ? 1 : 0; 	// LT (unsigned)
			ALUSLL:  Result <= B << A; 						 			// SLL
			ALUSRL:  Result <= B >> A; 						 			// SRL
			ALUSRA:  Result <= A >>> B; 					 			// SRA
			ALULUI:  Result <= B << 16; 	 	 						// LUI
			ALUJ: 	 Result <= (A & 'hF0000000) | B;  					// J (B has been << 2 and zero-extended)
			ALUNOP:  Result <= 0; 							 			// NOP
			default Result<='bx;
		endcase

endmodule

module RegisterFile #(parameter WIDTH = 32, REGBITS = 5, MEMUNITS = 4096)
					 (CLK, Reset, A1, A2, A3, RD1, RD2, WE3, WD3);
		
	input 	CLK, Reset, WE3;
	input 	[REGBITS-1:0] A1, A2, A3;
	input	[WIDTH-1:0] WD3;
	output  [WIDTH-1:0] RD1, RD2;
	
	reg [WIDTH-1:0] Registers [(1<<REGBITS)-1:0];
	
	parameter SP = 29;
	
	// reset all registers if needed
//	integer i;
//	always @(posedge CLK)
//		if (Reset) 
//			// set all registers to 0
//			for (i = 0; i < (1<<REGBITS); i = i + 1)
//				Registers[i] <= 0;
		
	// three ported register file
	// read two ports combinationally
	// write third port on positive edge of clock
	always @(posedge CLK)
		if (WE3) Registers[A3] <= WD3;
		
	// register 0 ($zero) hard-wired to 0
	assign RD1 = A1 ? Registers[A1] : 0;
	assign RD2 = A2 ? Registers[A2] : 0;
endmodule

module ZeroDetect #(parameter WIDTH = 32)
				   (A, Zero);
	
	input  [WIDTH-1:0] A;
	output reg		   Zero;
	
	always @(*)
		Zero <= (A == 0) ? 1 : 0;
	
endmodule

module SignDetect #(parameter WIDTH = 32)
				   (A, Sign);
				   
	input  [WIDTH-1:0] A;
	output reg		   Sign;

	always @(*)
		Sign <= A[WIDTH-1];
	
endmodule

module OverflowDetect #(parameter WIDTH = 32)
					   (Result, A, B, AluControl, Overflow);
					   
	input [WIDTH-1:0] Result, A, B;
	input 		[4:0] AluControl;
	output reg	      Overflow;
	
	// Overflow occurred when the sign of two operands is both positive (or negative)
	// but the sign of result is negative (or positive)
	
	// And in fact, only ADD, ADDI, SUB will validly set overflow flag
	always @(*)
		case (AluControl)
			5'b00000: Overflow <= ((A[WIDTH-1] == B[WIDTH-1]) && (A[WIDTH-1] != Result[WIDTH-1])) ? 1 : 0; // ADD, ADDI
			5'b00010: Overflow <= ((A[WIDTH-1] != B[WIDTH-1]) && (A[WIDTH-1] != Result[WIDTH-1])) ? 1 : 0; // SUB
			default: Overflow <= 1'bx; // set to unknown
		endcase
		
endmodule

module Flop #(parameter WIDTH = 32)
			 (CLK, D, Q);
			 
	input 					CLK;
	input 		[WIDTH-1:0] D;
	output reg  [WIDTH-1:0] Q;
			 
	always @(posedge CLK)
		Q <= D;
	
endmodule

module FlopEn #(parameter WIDTH = 32)
			   (CLK, En, D, Q);
			   
	input					CLK, En;
	input 		[WIDTH-1:0] D;
	output reg  [WIDTH-1:0] Q;
	
	always @(posedge CLK)
		if (En) Q <= D;
	
endmodule

module FlopEnReset #(parameter WIDTH = 32)
					(CLK, En, Reset, D, Q);
		
	input					CLK, En, Reset;
	input 		[WIDTH-1:0] D;
	output reg  [WIDTH-1:0] Q;
	
	always @(posedge CLK)
		if (Reset) Q <= 0;
		else if (En) Q <= D;
		
endmodule

module Mux2 #(parameter WIDTH = 32)
			 (D0, D1, Cond, Result);
	
	
	input			       Cond;
	input      [WIDTH-1:0] D0, D1;
	output     [WIDTH-1:0] Result;
			 
	assign Result = Cond ? D1 : D0;
			 
endmodule

module Mux4 #(parameter WIDTH = 32)
			 (D0, D1, D2, D3, Cond, Result);
		
	input	   		 [1:0] Cond;
	input      [WIDTH-1:0] D0, D1, D2, D3;
	output reg [WIDTH-1:0] Result;
	
	always @(*)
		case (Cond)
			2'b00: Result <= D0;
			2'b01: Result <= D1;
			2'b10: Result <= D2;
			2'b11: Result <= D3;
		endcase
		
endmodule

module Mux8 #(parameter WIDTH = 32)
			 (D0, D1, D2, D3, D4, D5, D6, D7, Cond, Result);
		
	input	   		 [2:0] Cond;
	input  	   [WIDTH-1:0] D0, D1, D2, D3, D4, D5, D6, D7;
	output reg [WIDTH-1:0] Result;
	
	always @(*)
		case (Cond)
			3'b000: Result <= D0;
			3'b001: Result <= D1;
			3'b010: Result <= D2;
			3'b011: Result <= D3;
			3'b100: Result <= D4;
			3'b101: Result <= D5;
			3'b110: Result <= D6;
			3'b111: Result <= D7;
		endcase
		
endmodule

module SignedCompare (Zero, Sign, Overflow, ALUSignCond, ALUCompare);
	
	input        Zero, Sign, Overflow;
	input  [2:0] ALUSignCond;
	
	output reg   ALUCompare;
	
	always @(*)
		case (ALUSignCond)
			// GT, ZF = 0 and SF = OF
			3'b000: ALUCompare <= ((~Zero) & (Sign ^~ Overflow));
			// LT, SF != OF
			3'b001: ALUCompare <= (Sign ^ Overflow);
			// EQ, ZF = 1
			3'b010: ALUCompare <= (Zero);
			// NOP
			3'b011: ALUCompare <= 1'bx;
			// NE, ZF = 0
			3'b100: ALUCompare <= (~Zero);
			// GE, SF = OF
			3'b101: ALUCompare <= (Sign ^~ Overflow);
			// LE, ZF = 1 or SF != OF
			3'b110: ALUCompare <= (Zero | (Sign ^ Overflow));
			// NOP
			3'b111: ALUCompare <= 1'bx;
		endcase
	
endmodule	