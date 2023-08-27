`ifndef LOG_FILE
    parameter LOG_FILE = "debug.log";
`endif

`ifndef LABEL_FILE
    parameter LABEL_FILE = "";
`endif

int log_fd, label_fd, Nlabels, i;
int inst_cnt=0;
string labels[], _lbl, rline;
int label_addrs[], _lbl_addr;
int scanRet;
int j1,j2;
initial begin
    log_fd = $fopen(LOG_FILE, "w");
    cpu_cycle = 0;
    inst_cnt = 0;
    
    Nlabels=0;
    if (LABEL_FILE != "") begin
        label_fd = $fopen(LABEL_FILE,"r");
        scanRet=1;
        while (!$feof(label_fd)) begin
            scanRet = $fscanf(label_fd, "%s %h, %d, %h", _lbl, _lbl_addr,j1,j2);
            Nlabels++;
        end
        $fclose(label_fd);
        label_fd = $fopen(LABEL_FILE,"r");
        label_addrs = new[Nlabels];
        labels = new[Nlabels];
        scanRet = 0;
        for (i=0; i<Nlabels; i++) begin
            scanRet = $fscanf(label_fd, "%s %h, %d, %h", _lbl, _lbl_addr,j1,j2);
            labels[i] = _lbl.substr(0,_lbl.len()-2); //trim comma
            label_addrs[i] = _lbl_addr;
        end
        $fclose(label_fd);
    end
end

logic upa,upx,upy,ups;
always @(posedge i_clk ) begin
    upa <= sb_a;
    upx <= sb_x;
    upy <= sb_y;
    ups <= push||pop;
end

logic [15:0] pc_r;
int arg_cnt=0;
always @(posedge i_clk ) begin

    pc_r <= pc;

    if (state==T1_DECODE) $fwrite( log_fd, " %s", op_name(opcode));
    if (inst_cnt>0 && pc != pc_r) begin
        $fwrite( log_fd, " %2h", i_data);
        arg_cnt = arg_cnt+1;
    end

    if (sync && inst_cnt>0) begin
        // reduce spaces depending on argument count to align reg dataus
        for(int p=0;p<5-arg_cnt;p=p+1) $fwrite( log_fd, "   ");
        $fwrite( log_fd, "A:%2h X:%2h Y:%2h P:%2h SP:%2h CYC:%0d\n", a,x,y,p,s, cpu_cycle);
    end

    if (sync) begin
        inst_cnt = inst_cnt+1;
        $fwrite( log_fd, "%4h", addr);
        arg_cnt = 0;
    end


    // $fwrite( log_fd, "%4h\t%s\tA:%2h X:%2h Y:%2h P:%2h SP:%2h CYC:%0d\n",
    //                                 pc,state_name(state), a,x,y,p,s, cpu_cycle);


    // $fwrite( log_fd, "\t%s:\taddr:%s,%s=%h db:%h(%s) sb:%h(%s) %sA:%2h X:%2h Y:%2h P:%2h SP:%2h CYC:%d\n" ,
    //                                 state_name(state),
    //                                 addr_name(adh_src), addr_name(adl_src), addr,
    //                                 db,db_name(db_src), sb,reg_name(sb_src), 
    //                                 a,x,y,p,sp, inst_cnt);


    // if (db_write) begin
    //     if (addr==16'h0F00)
    //         $fwrite( log_fd, "\tUART WRITE: '%c'\n", dor);
    //     else if (push)
    //         $fwrite( log_fd, "\tPUSH: 0x%h to 0x%h\n", dor, addr);
    //     else
    //         $fwrite( log_fd, "\tWRITE: 0x%h to 0x%h\n", dor, addr);
    // end
    // if (exec) begin
    //         alu_OP = op_alu_OP;
    //         ai_inv = op_ai_inv;
    //         bi_inv = op_bi_inv;
    //         ci = (op_Pci && p[0]) || op_ci;
    //         sb_src = op_sb_src;
    //         db_src = op_db_src;

    //     $fwrite( log_fd, "\tEXEC: %s\tai:%h bi:%h ci:%h out:%h\n", alu_name(alu_OP), ai, bi, ci, alu_out);
    // end
    // if (jump) begin
    //     _lbl = "nolabel";
    //     for (i=0; i<Nlabels; i++) begin
    //         if (label_addrs[i]==addr) begin
    //             _lbl = labels[i];
    //         end
    //     end        
    //     $fwrite( log_fd, "\tJUMPING to %s\n", format_addr(addr));
    // end
end    

final begin
    $fclose(log_fd);
end

function string format_addr(input int _addr);
    int base;
    string base_name;
    string name;
    base = 0;
    base_name = "0x0000";
    for (i=0; i<Nlabels; i++) begin
        if (_addr >= label_addrs[i] && label_addrs[i]>base) begin
            base = label_addrs[i];
            base_name = labels[i];
        end
    end
    // if (addr==base)
    //     $sformat(name, "%s", base_name);
    // else
        $sformat(name, "%s+0x%0h", base_name, _addr-base);

    format_addr = name;
endfunction

function string state_name(input logic[5:0] x);
    case(x)
        T0_FETCH: state_name = "T0_FCH";
        T1_DECODE: state_name = "T1_DCD";
        T2_ZPG: state_name = "T2_ZPG";
        T2_ZPGXY: state_name = "T2_ZXY";
        T3_ZPGXY: state_name = "T3_ZXY";
        T2_ABS: state_name = "T2_ABS";
        T3_ABS: state_name = "T3_ABS";
        T2_ABSXY: state_name = "T2_AXY";
        T3_ABSXY: state_name = "T3_AXY";
        T4_ABSXY: state_name = "T4_AXY";
        T2_XIND: state_name = "T2_XIN";
        T3_XIND: state_name = "T3_XIN";
        T4_XIND: state_name = "T4_XIN";
        T5_XIND: state_name = "T5_XIN";
        T2_INDY: state_name = "T2_INY";
        T3_INDY: state_name = "T3_INY";
        T4_INDY: state_name = "T4_INY";
        T5_INDY: state_name = "T5_INY";
        T_RMW_EXEC: state_name = "T_RWE";
        T_RMW_STORE: state_name = "T_RWS";
        T_BOOT: state_name = "T_BOOT";
        T_JAM: state_name = "T_JAM";
        T2_JUMP: state_name = "T2_JMP";
        T3_JUMP: state_name = "T3_JMP";
        T2_BRANCH: state_name = "T2_BRA";
        T3_BRANCH: state_name = "T3_BRA";
        T4_BRANCH: state_name = "T4_BRA";
        T2_PUSH: state_name = "T2_PSH";
        T2_POP: state_name = "T2_POP";
        T3_POP: state_name = "T3_POP";
        T2_BRK: state_name = "T2_BRK";
        T3_BRK: state_name = "T3_BRK";
        T4_BRK: state_name = "T4_BRK";
        T5_BRK: state_name = "T5_BRK";
        T2_RTI: state_name = "T2_RTI";
        T3_RTI: state_name = "T3_RTI";
        T4_RTI: state_name = "T4_RTI";
        T5_RTI: state_name = "T5_RTI";
        T2_RTS: state_name = "T2_RTS";
        T3_RTS: state_name = "T3_RTS";
        T4_RTS: state_name = "T4_RTS";
        T5_RTS: state_name = "T5_RTS";
        T2_JSR: state_name = "T2_JSR";
        T3_JSR: state_name = "T3_JSR";
        T4_JSR: state_name = "T4_JSR";
        T5_JSR: state_name = "T5_JSR";
        default: state_name = "UNDEF";
    endcase
endfunction

function string reg_name(input logic [2:0] x);
    case(x)
        REG_Z: reg_name = "Z  ";
        REG_A: reg_name = "A  ";
        REG_X: reg_name = "X  ";
        REG_Y: reg_name = "Y  ";
        REG_S: reg_name = "S  ";
        REG_P: reg_name = "P  ";
        REG_ADD: reg_name = "ADD";
        REG_DATA: reg_name = "DAT";
        default: reg_name = "UNDEF";
    endcase
endfunction

function string db_name(input logic [2:0] x);
    case(x)
        DB_Z: db_name = "Z  ";
        DB_DATA: db_name = "DAT";
        DB_PCL: db_name = "PCL";
        DB_PCH: db_name = "PCH";
        DB_S: db_name = "S  ";
        DB_P: db_name = "P  ";
        DB_A: db_name = "A  ";
        DB_SB: db_name = "SB ";
        default: db_name = "UNDEF";
    endcase
endfunction


function string addr_name(input logic [2:0] x);
    case(x)
        ADDR_PC: addr_name = "PC ";
        ADDR_DATA: addr_name = "DAT";
        ADDR_RES: addr_name = "RES";
        ADDR_ADD: addr_name = "ADD";
        ADDR_Z: addr_name = "Z  ";
        ADDR_HOLD: addr_name = "HLD"; 
        ADDR_INT: addr_name = "INT"; 
        ADDR_STACK: addr_name = "STK"; 
        default: addr_name = "UNDEF";
    endcase
endfunction

function string alu_name(input logic [2:0] x);
    case(x)
        ALU_NOP: alu_name = "NOP";
        ALU_ADD: alu_name = "ADD";
        ALU_AND: alu_name = "AND";
        ALU_OR: alu_name = "OR ";
        ALU_XOR: alu_name = "XOR";
        ALU_SR: alu_name = "SR ";
        ALU_SL: alu_name = "SL ";
        ALU_BIT: alu_name = "BIT";
        default: alu_name = "UNDEF";
    endcase
endfunction


function string op_name(input logic [7:0] op);
    case(opcode)
        8'h00: op_name = "BRK";
        8'h01: op_name = "ORA";
        8'h02: op_name = "*KIL";
        8'h03: op_name = "*SLO";
        8'h04: op_name = "*NOP";
        8'h05: op_name = "ORA";
        8'h06: op_name = "ASL";
        8'h07: op_name = "*SLO";
        8'h08: op_name = "PHP";
        8'h09: op_name = "ORA";
        8'h0A: op_name = "ASL";
        8'h0B: op_name = "*ANC";
        8'h0C: op_name = "*NOP";
        8'h0D: op_name = "ORA";
        8'h0E: op_name = "ASL";
        8'h0F: op_name = "*SLO";
        8'h10: op_name = "BPL";
        8'h11: op_name = "ORA";
        8'h12: op_name = "*KIL";
        8'h13: op_name = "*SLO";
        8'h14: op_name = "*NOP";
        8'h15: op_name = "ORA";
        8'h16: op_name = "ASL";
        8'h17: op_name = "*SLO";
        8'h18: op_name = "CLC";
        8'h19: op_name = "ORA";
        8'h1A: op_name = "*NOP";
        8'h1B: op_name = "*SLO";
        8'h1C: op_name = "*NOP";
        8'h1D: op_name = "ORA";
        8'h1E: op_name = "ASL";
        8'h1F: op_name = "*SLO";
        8'h20: op_name = "JSR";
        8'h21: op_name = "AND";
        8'h22: op_name = "*KIL";
        8'h23: op_name = "*RLA";
        8'h24: op_name = "BIT";
        8'h25: op_name = "AND";
        8'h26: op_name = "ROL";
        8'h27: op_name = "*RLA";
        8'h28: op_name = "PLP";
        8'h29: op_name = "AND";
        8'h2A: op_name = "ROL";
        8'h2B: op_name = "*ANC";
        8'h2C: op_name = "BIT";
        8'h2D: op_name = "AND";
        8'h2E: op_name = "ROL";
        8'h2F: op_name = "*RLA";
        8'h30: op_name = "BMI";
        8'h31: op_name = "AND";
        8'h32: op_name = "*KIL";
        8'h33: op_name = "*RLA";
        8'h34: op_name = "*NOP";
        8'h35: op_name = "AND";
        8'h36: op_name = "ROL";
        8'h37: op_name = "*RLA";
        8'h38: op_name = "SEC";
        8'h39: op_name = "AND";
        8'h3A: op_name = "*NOP";
        8'h3B: op_name = "*RLA";
        8'h3C: op_name = "*NOP";
        8'h3D: op_name = "AND";
        8'h3E: op_name = "ROL";
        8'h3F: op_name = "*RLA";
        8'h40: op_name = "RTI";
        8'h41: op_name = "EOR";
        8'h42: op_name = "*KIL";
        8'h43: op_name = "*SRE";
        8'h44: op_name = "*NOP";
        8'h45: op_name = "EOR";
        8'h46: op_name = "LSR";
        8'h47: op_name = "*SRE";
        8'h48: op_name = "PHA";
        8'h49: op_name = "EOR";
        8'h4A: op_name = "LSR";
        8'h4B: op_name = "*ALR";
        8'h4C: op_name = "JMP";
        8'h4D: op_name = "EOR";
        8'h4E: op_name = "LSR";
        8'h4F: op_name = "*SRE";
        8'h50: op_name = "BVC";
        8'h51: op_name = "EOR";
        8'h52: op_name = "*KIL";
        8'h53: op_name = "*SRE";
        8'h54: op_name = "*NOP";
        8'h55: op_name = "EOR";
        8'h56: op_name = "LSR";
        8'h57: op_name = "*SRE";
        8'h58: op_name = "CLI";
        8'h59: op_name = "EOR";
        8'h5A: op_name = "*NOP";
        8'h5B: op_name = "*SRE";
        8'h5C: op_name = "*NOP";
        8'h5D: op_name = "EOR";
        8'h5E: op_name = "LSR";
        8'h5F: op_name = "*SRE";
        8'h60: op_name = "RTS";
        8'h61: op_name = "ADC";
        8'h62: op_name = "*KIL";
        8'h63: op_name = "*RRA";
        8'h64: op_name = "*NOP";
        8'h65: op_name = "ADC";
        8'h66: op_name = "ROR";
        8'h67: op_name = "*RRA";
        8'h68: op_name = "PLA";
        8'h69: op_name = "ADC";
        8'h6A: op_name = "ROR";
        8'h6B: op_name = "*ARR";
        8'h6C: op_name = "JMP";
        8'h6D: op_name = "ADC";
        8'h6E: op_name = "ROR";
        8'h6F: op_name = "*RRA";
        8'h70: op_name = "BVS";
        8'h71: op_name = "ADC";
        8'h72: op_name = "*KIL";
        8'h73: op_name = "*RRA";
        8'h74: op_name = "*NOP";
        8'h75: op_name = "ADC";
        8'h76: op_name = "ROR";
        8'h77: op_name = "*RRA";
        8'h78: op_name = "SEI";
        8'h79: op_name = "ADC";
        8'h7A: op_name = "*NOP";
        8'h7B: op_name = "*RRA";
        8'h7C: op_name = "*NOP";
        8'h7D: op_name = "ADC";
        8'h7E: op_name = "ROR";
        8'h7F: op_name = "*RRA";
        8'h80: op_name = "*NOP";
        8'h81: op_name = "STA";
        8'h82: op_name = "*NOP";
        8'h83: op_name = "*SAX";
        8'h84: op_name = "STY";
        8'h85: op_name = "STA";
        8'h86: op_name = "STX";
        8'h87: op_name = "*SAX";
        8'h88: op_name = "DEY";
        8'h89: op_name = "*NOP";
        8'h8A: op_name = "TXA";
        8'h8B: op_name = "*XAA";
        8'h8C: op_name = "STY";
        8'h8D: op_name = "STA";
        8'h8E: op_name = "STX";
        8'h8F: op_name = "*SAX";
        8'h90: op_name = "BCC";
        8'h91: op_name = "STA";
        8'h92: op_name = "*KIL";
        8'h93: op_name = "*AHX";
        8'h94: op_name = "STY";
        8'h95: op_name = "STA";
        8'h96: op_name = "STX";
        8'h97: op_name = "*SAX";
        8'h98: op_name = "TYA";
        8'h99: op_name = "STA";
        8'h9A: op_name = "TXS";
        8'h9B: op_name = "*TAS";
        8'h9C: op_name = "*SHY";
        8'h9D: op_name = "STA";
        8'h9E: op_name = "*SHX";
        8'h9F: op_name = "*AHX";
        8'hA0: op_name = "LDY";
        8'hA1: op_name = "LDA";
        8'hA2: op_name = "LDX";
        8'hA3: op_name = "*LAX";
        8'hA4: op_name = "LDY";
        8'hA5: op_name = "LDA";
        8'hA6: op_name = "LDX";
        8'hA7: op_name = "*LAX";
        8'hA8: op_name = "TAY";
        8'hA9: op_name = "LDA";
        8'hAA: op_name = "TAX";
        8'hAB: op_name = "*LAX";
        8'hAC: op_name = "LDY";
        8'hAD: op_name = "LDA";
        8'hAE: op_name = "LDX";
        8'hAF: op_name = "*LAX";
        8'hB0: op_name = "BCS";
        8'hB1: op_name = "LDA";
        8'hB2: op_name = "*KIL";
        8'hB3: op_name = "*LAX";
        8'hB4: op_name = "LDY";
        8'hB5: op_name = "LDA";
        8'hB6: op_name = "LDX";
        8'hB7: op_name = "*LAX";
        8'hB8: op_name = "CLV";
        8'hB9: op_name = "LDA";
        8'hBA: op_name = "TSX";
        8'hBB: op_name = "*LAS";
        8'hBC: op_name = "LDY";
        8'hBD: op_name = "LDA";
        8'hBE: op_name = "LDX";
        8'hBF: op_name = "*LAX";
        8'hC0: op_name = "CPY";
        8'hC1: op_name = "CMP";
        8'hC2: op_name = "*NOP";
        8'hC3: op_name = "*DCP";
        8'hC4: op_name = "CPY";
        8'hC5: op_name = "CMP";
        8'hC6: op_name = "DEC";
        8'hC7: op_name = "*DCP";
        8'hC8: op_name = "INY";
        8'hC9: op_name = "CMP";
        8'hCA: op_name = "DEX";
        8'hCB: op_name = "*AXS";
        8'hCC: op_name = "CPY";
        8'hCD: op_name = "CMP";
        8'hCE: op_name = "DEC";
        8'hCF: op_name = "*DCP";
        8'hD0: op_name = "BNE";
        8'hD1: op_name = "CMP";
        8'hD2: op_name = "*KIL";
        8'hD3: op_name = "*DCP";
        8'hD4: op_name = "*NOP";
        8'hD5: op_name = "CMP";
        8'hD6: op_name = "DEC";
        8'hD7: op_name = "*DCP";
        8'hD8: op_name = "CLD";
        8'hD9: op_name = "CMP";
        8'hDA: op_name = "*NOP";
        8'hDB: op_name = "*DCP";
        8'hDC: op_name = "*NOP";
        8'hDD: op_name = "CMP";
        8'hDE: op_name = "DEC";
        8'hDF: op_name = "*DCP";
        8'hE0: op_name = "CPX";
        8'hE1: op_name = "SBC";
        8'hE2: op_name = "*NOP";
        8'hE3: op_name = "*ISC";
        8'hE4: op_name = "CPX";
        8'hE5: op_name = "SBC";
        8'hE6: op_name = "INC";
        8'hE7: op_name = "*ISC";
        8'hE8: op_name = "INX";
        8'hE9: op_name = "SBC";
        8'hEA: op_name = "NOP";
        8'hEB: op_name = "*SBC";
        8'hEC: op_name = "CPX";
        8'hED: op_name = "SBC";
        8'hEE: op_name = "INC";
        8'hEF: op_name = "*ISC";
        8'hF0: op_name = "BEQ";
        8'hF1: op_name = "SBC";
        8'hF2: op_name = "*KIL";
        8'hF3: op_name = "*ISC";
        8'hF4: op_name = "*NOP";
        8'hF5: op_name = "SBC";
        8'hF6: op_name = "INC";
        8'hF7: op_name = "*ISC";
        8'hF8: op_name = "SED";
        8'hF9: op_name = "SBC";
        8'hFA: op_name = "*NOP";
        8'hFB: op_name = "*ISC";
        8'hFC: op_name = "*NOP";
        8'hFD: op_name = "SBC";
        8'hFE: op_name = "INC";
        8'hFF: op_name = "*ISC";
        default: op_name = "UNDEF";
    endcase
endfunction
