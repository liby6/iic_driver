`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: FMSH
// Engineer: lihairong 
// 
// Create Date: 2022/05/14 17:56:16
// Design Name: 
// Module Name: iic_driver
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module iic_driver #
	(
		parameter	CLKIN_MHz = 100,  	//default clock in frequency is  100MHz
		parameter 	IIC_KHz = 1000,  	//default IIC scl frequency is 100KHz 
		parameter	TIMEOUT_us = 1,		//default wait slave ask time is 1us
		parameter	RESTART_DELAY_us = 0 //default read operation restart delay time is 0us
	)
	(
	
	input 				clk_in,
	input 				rst_n,
	
	input 				sda_in,
	output	reg			sda_out,
	output	reg			sda_en,  	//high-->output
	output	reg			scl,	 	//iIC clock output
	
	input				cmd_en,  	//begin translate IIC when detected cmd_en posedge
	input	[7:0]		addr, 	 	//bit[7:1]-->iic_physicl
	input	[31:0]		cmd,	 	//pmbus cmd or eeprom operation addr
	input	[2:0]		cmd_len,	//cmd[31:0] valid byte num  --> (0|1|2|3|4)
	input				wdata_vld, 	//wdata_vld high index wdata valid  
	input	[7:0]		wdata,		//begin input wdata when  cmd_en valid <= 32byte

	input	[7:0]		rd_len,  	//read data len
	output	reg [7:0]	rdata, 		//read data form slave
	output	reg			rdata_vld,	//index read data valid 
	
	output	reg			cmd_done,	//high --> cmd finish
	output	reg 		opt_err		//hing --> wait ask timeout
			

    );
    
localparam  DATA_POS 	= CLKIN_MHz*1000 / (4*IIC_KHz) - 1,
    		CLK_POS 	= CLKIN_MHz*1000 / (2*IIC_KHz) - 1,
    		DATA_NEG 	= 3*CLKIN_MHz*1000 / (4*IIC_KHz) - 1,
			CLK_NEG 	= CLKIN_MHz*1000 / (IIC_KHz) - 1;
			
localparam  TIMEOUT = CLKIN_MHz * TIMEOUT_us + CLK_POS;
localparam	RESTART_DELAY = CLKIN_MHz * RESTART_DELAY_us;			
     
localparam  MAX_WR_LEN = 32;
    
localparam	IDLE 		= 4'd0,
			START 		= 4'd1,
			WRITE 		= 4'd2,
			WAIT_ASK	= 4'd3,
			DELAY		= 4'd4,
			READ		= 4'd5,
			ASK			= 4'd6,
			STOP		= 4'd7,
			FINISH		= 4'd8,
			ERR			= 4'd9;
				
localparam  DIR_READ	= 1'b1,
			DIR_WRITE	= 1'b0;
			
localparam 	OPT_IDLE	= 3'd0,
			ADDR1_DONE 	= 3'd1,
			CMD_DONE	= 3'd2,
			ADDR2_DONE	= 3'd3,
			WDATA_DONE	= 3'd4,
			RDATA_DONE	= 3'd5;
			
			
reg			cnt_flag;
reg 		dir_flag;
reg 		cmd_en_z; 
reg [2:0] 	opt_sta = OPT_IDLE;	
reg [2:0] 	opt_sta_z;		
reg [3:0] 	cur_sta = IDLE,nxt_sta = IDLE;
reg [31:0] 	timeout_cnt;
reg [31:0] 	delay_cnt;
reg [3:0] 	bit_cnt;       
reg [31:0] 	clk_cnt;
reg [7:0] 	byte_cnt;
reg [7:0]	byte_len;
reg [31:0]	dly_cnt;
reg [7:0]	wr_len;
reg [7:0] 	wdata_z;
reg [7:0]   wr_mem[MAX_WR_LEN - 1:0];


//clk_cnt
always@(posedge clk_in or negedge rst_n)
begin
	if(~rst_n)
		clk_cnt <= 0;
	else if(cur_sta == IDLE || clk_cnt == CLK_NEG)
		clk_cnt <= 0;
	else
	begin
		if(cnt_flag)
			clk_cnt <= clk_cnt + 1'b1;
		else
			clk_cnt <= clk_cnt;
	end
end

//timeout cnt
always@(posedge clk_in or negedge rst_n)
begin
	if(~rst_n)
		timeout_cnt <= 0;
	else
	begin
		if(nxt_sta == WAIT_ASK) 
			timeout_cnt <= timeout_cnt + 1'b1;
		else
			timeout_cnt <= 0;
	end
end

//restat delay cnt
always@(posedge clk_in or negedge rst_n)
begin
	if(~rst_n)
		delay_cnt <= 0;
	else
	begin
		if(nxt_sta == DELAY) 
			delay_cnt <= delay_cnt + 1'b1;
		else
			delay_cnt <= 0;
	end
end

//index clk cnt
always@(posedge clk_in or negedge rst_n)
begin
	if(~rst_n)
		cnt_flag <= 1'b1;
	else if((nxt_sta == WAIT_ASK) && (clk_cnt >= DATA_NEG))
	begin
		if(timeout_cnt >= TIMEOUT)
			cnt_flag <= 1'b1;
		else if(sda_in == 1'b1)
			cnt_flag <= 1'b0;
		else
			cnt_flag <= 1'b1;
	end
	else if(nxt_sta == DELAY)
	begin
		if(delay_cnt > RESTART_DELAY)
			cnt_flag <= 1'b1;
		else
			cnt_flag <= 1'b0;
	end
	else
	begin
		cnt_flag <= 1'b1;
	end
end

//detect cmd_en posedge    
always@(posedge clk_in or negedge rst_n)
begin
	if(~rst_n)
		cmd_en_z <= 8'h0;
	else
		cmd_en_z <= cmd_en;
end

//addr[0]-->WRITE or READ
always@(posedge clk_in or negedge rst_n)
begin
	if(~rst_n)
		dir_flag <= 1'h0;
	else if(cmd_en && ~cmd_en_z)//detect cmd_en posedge 
		dir_flag <= addr[0];
	else
		dir_flag <= dir_flag;
end


//cnt wrdata len
always@(posedge clk_in or negedge rst_n)
begin
	if(~rst_n)
		wr_len <= 8'h0;
	else if(wdata_vld && wr_len < MAX_WR_LEN) 
		wr_len <= wr_len + 1'b1;
	else if(nxt_sta == IDLE)
		wr_len <= 8'h0;
	else
		wr_len <= wr_len;
end

//storage wrdata
integer i;
always@(posedge clk_in or negedge rst_n)
begin
	if(~rst_n)
	begin
		for(i = 0; i < MAX_WR_LEN; i = i+1)
			wr_mem[i] <= 8'h0;
	end
	else if(wdata_vld) 
		wr_mem[wr_len] <= wdata;
	else
		wr_mem[wr_len] <= wr_mem[wr_len];
end
 
//write data or read data cnt bit
always@(posedge clk_in or negedge rst_n)
begin
	if(~rst_n)
		bit_cnt <= 4'b0;
	else
	begin
		if(nxt_sta == WRITE || nxt_sta == READ)
		begin
			if(clk_cnt == DATA_POS)
				bit_cnt <= bit_cnt + 1'b1;
			else
				bit_cnt <= bit_cnt;
		end
		else
			bit_cnt <= 4'b0;
	end
end




//opt_sta index state change
always@(posedge clk_in or negedge rst_n)
begin
	if(~rst_n)
		opt_sta <= OPT_IDLE;
	else if(cur_sta == IDLE)
		opt_sta <= OPT_IDLE;
	else if(opt_sta == OPT_IDLE && cur_sta == WRITE && nxt_sta == WAIT_ASK) //write addr done
	begin
		if(cmd_len == 0) //no cmd 
			opt_sta <= (dir_flag == DIR_READ)?ADDR2_DONE:CMD_DONE;
		else
			opt_sta <= ADDR1_DONE;
	end
	else if(opt_sta == ADDR1_DONE && cur_sta == WRITE && nxt_sta == WAIT_ASK && byte_len == byte_cnt) 
	begin
		if(dir_flag == DIR_WRITE && wr_len == 0)
			opt_sta <= WDATA_DONE;
		else
			opt_sta <= CMD_DONE;
	end
	else if(opt_sta == CMD_DONE && cur_sta == WRITE && nxt_sta == WAIT_ASK && dir_flag == DIR_READ)
		opt_sta <= ADDR2_DONE;
	else if(opt_sta == CMD_DONE && cur_sta == WRITE && nxt_sta == WAIT_ASK && byte_len == byte_cnt)
		opt_sta <= WDATA_DONE;
	else if(opt_sta == ADDR2_DONE && cur_sta == READ && nxt_sta == ASK && byte_len == byte_cnt)
		opt_sta <= RDATA_DONE;
	else
		opt_sta <= opt_sta;
end

//index clear byte cnt
always@(posedge clk_in or negedge rst_n)
begin
	if(~rst_n)
		opt_sta_z <= 3'b0;
	else
		opt_sta_z <= opt_sta;
end

//byte cnt
always@(posedge clk_in or negedge rst_n)
begin
	if(~rst_n)
		byte_cnt <= 8'b0;
	else if(opt_sta_z != opt_sta)
		byte_cnt <= 8'b0;
	else
	begin
		if((nxt_sta == WRITE || nxt_sta == READ ) && (clk_cnt == CLK_POS) && (bit_cnt == 8))
			byte_cnt <= byte_cnt + 1'b1;
		else
			byte_cnt <= byte_cnt;
	end
end

//byte len
always@(posedge clk_in or negedge rst_n)
begin
	if(~rst_n)
		byte_len <= 8'b0;
	else if(nxt_sta == START || nxt_sta == STOP)
		byte_len <= 8'd0;
	else if(cur_sta == START && nxt_sta == WRITE) //addr len --> 1
		byte_len <= 8'd1;
	else if(opt_sta == ADDR1_DONE && nxt_sta == WAIT_ASK )
		byte_len <= cmd_len;
	else if(opt_sta == CMD_DONE && nxt_sta == WAIT_ASK)
		byte_len <= wr_len;
	else if(opt_sta == ADDR2_DONE && nxt_sta == WAIT_ASK )
		byte_len <= rd_len;
	else
		byte_len <= byte_len;
end



//wdata
always@(posedge clk_in or negedge rst_n)
begin
	if(~rst_n)
		wdata_z <= 8'b0;
	else if(opt_sta == IDLE)
		wdata_z <= {addr[7:1],DIR_WRITE};
	else if(cur_sta == WAIT_ASK && nxt_sta == START)
		wdata_z <= {addr[7:1],DIR_READ};
	else if(opt_sta == ADDR1_DONE) 
	begin
		case(byte_cnt)
		0:wdata_z <= cmd[7:0];
		1:wdata_z <= cmd[15:8];
		2:wdata_z <= cmd[23:16];
		default:wdata_z <= cmd[31:24];
		endcase
	end
	else if(opt_sta == CMD_DONE && dir_flag == DIR_WRITE)
		wdata_z <= wr_mem[byte_cnt];
	else
		wdata_z <= wdata_z;
end


//iic scl
always@(posedge clk_in or negedge rst_n)
begin
	if(~rst_n)
		scl <= 1'b1;
	else if(nxt_sta == IDLE || (cur_sta == WAIT_ASK && nxt_sta == START))
		scl <= 1'b1;
	else if(nxt_sta == STOP || nxt_sta == DELAY)
	begin
		if(clk_cnt == CLK_POS)
			scl <= 1'b1;
		else
			scl <= scl;
	end
	else
	begin
		if(clk_cnt == CLK_POS)
			scl <= 1'b1;
		else if(clk_cnt == CLK_NEG)
			scl <= 1'b0;
	end
end

//iic sda output
always@(posedge clk_in or negedge rst_n)
begin
	if(~rst_n)
	begin
		sda_out <= 1'b1;
	end
	else
	begin
		if(nxt_sta == IDLE)
		begin
			sda_out <= 1'b1;
		end
		else if(nxt_sta == START) 	
		begin
			if(clk_cnt == CLK_POS)
				sda_out <= 1'b0;
			else
				sda_out <= sda_out;
		end
		else if(nxt_sta == WRITE)
		begin
			if(clk_cnt == DATA_POS)
				sda_out <= wdata_z[7- bit_cnt];
			else
				sda_out <= sda_out;
		end
		else if(nxt_sta == DELAY)
		begin
			sda_out <= 1'b1;
		end
		else if(nxt_sta == ASK)
		begin
			if(clk_cnt == DATA_POS)
			begin
				if(byte_len == byte_cnt)
					sda_out <= 1'b1;
				else
					sda_out <= 1'b0;
			end
		end
		else if(nxt_sta == READ || nxt_sta == WAIT_ASK)
		begin
			sda_out <= 1'b1;
		end
		else if(nxt_sta == STOP)
		begin
			if(clk_cnt == DATA_POS)
				sda_out <= 1'b0;
			else if(clk_cnt == CLK_NEG)
				sda_out <= 1'b1;
		end
		else
		begin
			sda_out <= sda_out;
		end
	end
end


//iic sda output enable
always@(posedge clk_in or negedge rst_n)
begin
	if(~rst_n)
	begin
		sda_en <= 1'b1;
	end
	else
	begin
		if(nxt_sta == READ || nxt_sta == WAIT_ASK)
			sda_en <= 1'b0;
		else
			sda_en <= 1'b1;
	end
end


//read data from slave
always@(posedge clk_in or negedge rst_n)
begin
	if(~rst_n)
		rdata <= 8'b0;
	else if(nxt_sta == READ)
	begin
		if(clk_cnt == CLK_NEG)
			rdata <= {rdata[6:0],sda_in};
		else
			rdata <= rdata;
	end
	else
		rdata <= 8'b0;
end


//read data valid 
always@(posedge clk_in or negedge rst_n)
begin
	if(~rst_n)
		rdata_vld <= 1'b0;
	else if((cur_sta == READ) && (nxt_sta == ASK))
		rdata_vld <= 1'b1;
	else
		rdata_vld <= 1'b0;

end


always@(posedge clk_in or negedge rst_n)
begin
	if(~rst_n)
		cmd_done <= 1'b0;
	else if(cmd_en && !cmd_en_z)
		cmd_done <= 1'b0;
	else if(nxt_sta == FINISH)
		cmd_done <= 1'b1;
	else
		cmd_done <= cmd_done;
end


    
always@(posedge clk_in or negedge rst_n)
begin
	if(~rst_n)
		opt_err <= 1'b0;
	else if(cmd_en && !cmd_en_z)
		opt_err <= 1'b0;
	else if(timeout_cnt >= TIMEOUT) //(nxt_sta == ERR)
		opt_err <= 1'b1;
	else
		opt_err <= opt_err;
end




always@(posedge clk_in or negedge rst_n)
begin
	if(~rst_n)
		cur_sta <= IDLE;
	else
		cur_sta <= nxt_sta;
end

//Combinational logic
always@(*)
begin
	case(cur_sta)
		IDLE:
			begin
				if(cmd_en && !cmd_en_z)   //else latches
					nxt_sta = START;
			end			
		START: 	if(clk_cnt == CLK_NEG) 	//else latches
					nxt_sta = WRITE;	
		WRITE:	if((clk_cnt == DATA_POS) && (bit_cnt == 8)) //else null
					nxt_sta = WAIT_ASK;
		WAIT_ASK:
			begin
				if(clk_cnt == DATA_POS) //else latches
				begin
					if(timeout_cnt < TIMEOUT)
					begin
						if(opt_sta == CMD_DONE && dir_flag == DIR_READ)
							nxt_sta = DELAY;
						else if(opt_sta == ADDR2_DONE)
							nxt_sta = READ;
						else if(opt_sta == WDATA_DONE)
							nxt_sta = STOP;
						else
							nxt_sta = WRITE;
					end
					else
					nxt_sta = STOP;//ERR
				end
			end
		DELAY:	if(clk_cnt == DATA_POS) //else latches
					nxt_sta = START;
			
		READ:	if( (clk_cnt == DATA_POS) && (bit_cnt == 8)) //else latches
					nxt_sta = ASK;
		ASK:
			begin
				if(clk_cnt == DATA_POS)	//else latches
				begin
					if(opt_sta == RDATA_DONE)
						nxt_sta = STOP;
					else
						nxt_sta = READ;
				end
			end
		STOP:	if(clk_cnt == DATA_POS) //else latches
					nxt_sta = FINISH;	
		FINISH: nxt_sta = IDLE;
//		ERR:	nxt_sta = IDLE;	
		default:nxt_sta = IDLE;
		endcase
end    
    
endmodule
