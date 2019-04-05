module snake_game
	(
		PS2_KBCLK,                            // Keyboard clock
	   PS2_KBDAT,                            // Keyboard input data
		CLOCK_50,						//	On Board 50 MHz
      KEY,
		SW,
		LEDR,  // debug
		HEX0, // debug
		HEX1,
		HEX2,
		HEX5,
		HEX6,
		HEX7,
		// The ports below are for the VGA output.  Do not change.
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,						//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B   						//	VGA Blue[9:0]
	);

	input PS2_KBCLK;                            // Keyboard clock
	input PS2_KBDAT; 
	input	CLOCK_50;				//	50 MHz
	input [3:0]KEY;
	input [4:0]SW;
	output reg [5:0] LEDR; // debug
	output [6:0]HEX0, HEX1, HEX2, HEX5, HEX6, HEX7; // debug
	// Declare your inputs and outputs here
	// Do not change the following outputs
	output			VGA_CLK;   				//	VGA Clock
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;				//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[9:0]	VGA_R;   				//	VGA Red[9:0]
	output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
	output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
	
	wire clk, move, direction_clk;
	wire [3:0] direction;
	wire [3:0] hex2, hex1, hex0,hex3,hex5, hex6, hex7;
	wire [3:0] direction_control;
	
	assign hex2[3:0] = (score/100);
	assign hex1[3:0] = ((score%100)/10);
	assign hex0[3:0] = ((score%100)%10);
	hex_decoder h0(hex0, HEX0);
	hex_decoder h1(hex1, HEX1);
	hex_decoder h2(hex2, HEX2);
	
	assign hex7[3:0] = (max_score/100);
	assign hex6[3:0] = ((max_score%100)/10);
	assign hex5[3:0] = ((max_score%100)%10);
	hex_decoder h5(hex5, HEX5);
	hex_decoder h6(hex6, HEX6);
	hex_decoder h7(hex7, HEX7);
	//assign hex3[3:0] = score;
	//hex_decoder h3(hex3, HEX3);
	assign clk = CLOCK_50;
	assign reset = SW[0];
	reg [7:0] x, appleX;
	reg [6:0] y, appleY;
	reg [5:0] apple_xcounter, apple_ycounter;
	reg [2:0] color;
	reg [6:0] size, score, max_score; // snake have maximum 128 length
	reg [7:0] snakeX[0:127]; // 128 length array of 8 bit x
	reg [6:0] snakeY[0:127]; // 128 length array of 7 bit y
	reg is_snakeHead, is_snakeBody, is_apple, is_border;
	reg next_apple, game_over;
	
	integer i, j, k, n; // define some counters
	
	Speed_control Sc(clk, size, move);
	Direction_control Dc(clk, direction_control, reset, direction);
	
	vga_adapter VGA(
			.resetn(reset),
			.clock(clk),
			.colour(color),
			.x(x),
			.y(y),
			.plot(1), // write enable
			// Signals for the DAC to drive the monitor
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "black.mif";	
	
	wire [7:0] kb_scan_code;
	wire kb_sc_ready, kb_letter_case;
	key2ascii SC2A (
		.direction_control(direction_control),
		.scan_code(kb_scan_code),
		.letter_case(kb_letter_case)
	);
	keyboard kd (
		.clk(clk),
		.reset(~reset),
		.ps2d(PS2_KBDAT),
		.ps2c(PS2_KBCLK),
		.scan_code(kb_scan_code),
		.scan_code_ready(kb_sc_ready),
		.letter_case_out(kb_letter_case)
	);
	
	
	// increment x, y
	always @(posedge clk)
	begin
		if (reset)
		begin
				x <= x+1;
				if (x == 160 && y!=120) begin
					x <= 0;
					y <= y+1;
				end
				if (x == 160 && y == 120) begin
					x <= 0;
					y <= 0;
				end
		end else begin
				x <= 0;
				y <= 0;
		end
	end

	// apple production
	/////////////////////////////////////////////////////////
	always@(posedge clk)
	begin
		if(~reset)
			begin
				appleX <= 50;
				appleY <= 40;
				apple_xcounter = 0;
				apple_ycounter = 10;
				score <= 7'b0;
			end
		else if (next_apple)
		begin
				score <= score + 1;
			if ((apple_xcounter*2 + 15 <= 0) || (apple_xcounter*2 + 15 >= 156))
				begin
					appleX <= 120;
					appleY <= apple_ycounter*2;
				end
			else if ((apple_ycounter*2 <= 1) || (apple_ycounter*2 >= 118))
				begin
					appleX <= apple_xcounter*2 + 15;
					appleY <= 100;
				end
			else
				begin
						appleX <= apple_xcounter*2 + 15;
						appleY <= apple_ycounter*2;
				end
		end
		else
			begin
				apple_xcounter <= apple_xcounter + 1;
				apple_ycounter <= apple_ycounter + 1;
			end
	end
	//////////////////////////////////////////////////////
	
	///////////////////////////////////////////////////////
	// snake movement////// works!/// 
	always@(posedge move)
	begin
		LEDR[0] = ~LEDR[0]; //debug
		if(reset)
		begin
			// shift the body
			for(i = 127; i > 0; i = i - 1)
			begin
					if(i <= size - 1)
					begin
						snakeX[i] = snakeX[i - 1];
						snakeY[i] = snakeY[i - 1];
					end
			end
			// update headmove
			snakeX[0] <= (snakeX[0] + direction[0] - direction[3]);
			snakeY[0] <= (snakeY[0] + direction[1] - direction[2]);
		end
	
		else if(~reset)
		begin
			for(j = 1; j < 128; j = j+1)
				begin
				snakeX[j] = 161; // out of the screen
				snakeY[j] = 121; // out of the screen
				end
			snakeX[0] = 20;
			snakeY[0] = 15;
		end
	end
	
		
	// snake grow///////////
	always @(posedge clk) 
	begin
			next_apple <= 0;
			if((is_apple && is_snakeHead) && reset)
				begin
					next_apple<=1 ;
					if (size == 128) size <= size;
					else size <= size + 3;
				end
			else if(~reset)
				begin
					if (SW[1] && ~SW[2]) size <= 29;
					else if (SW[2] && ~SW[1]) size <= 47;
					else if (SW[1] && SW[2]) size <= 68;
					else size <= 8;
				end

	end
	///////////////////////////////////////////////////////
	
	///////////////////////////////////////////////////////
	// define the border
	always @(posedge clk)
	begin
		is_border <= 0;
		if ((x <= 0) || (x >= 156) || (y <= 0) || (y >= 119))
			is_border <= 1;
	end
	
	// check head
	always@(posedge clk)
	begin	
		is_snakeHead = (x == snakeX[0]) && (y == snakeY[0]);
	end
	
	// check body
	always@(posedge clk)
	begin
		is_snakeBody = 0;
		for(k = 1; k < size; k = k + 1)
		begin
			if(~is_snakeBody)
			begin				
				is_snakeBody = (x == snakeX[k]) && (y == snakeY[k]);
			end
		end
	end
	
	// check apple
	always@(posedge clk)
	begin	
		is_apple = (x == appleX) && (y == appleY);

	end

	// game over
	always @(posedge clk)
	begin
			LEDR[1] = game_over;
			if((is_border || is_snakeBody) && is_snakeHead) 
			begin
				game_over <= 1;
			end
			if (~reset) 
				game_over <= 0;
	end
	
	always @(posedge game_over)
	begin
		if (score > max_score) max_score <= score;
	end
	/////////////////////////////////////////////////////////////
									
	// assign color
	always @(posedge clk)
	begin
		
		if (is_apple && ~game_over) color <= 100;
		else if (game_over) color <= 001;
		else if ((is_snakeHead||is_snakeBody) && ~game_over) color <= 010;
		else if (is_border && ~game_over) color <= 011;
		else color <= 000;
	
	end
endmodule


// work!!!! ///
module Direction_control(clk, direction_control, reset, direction);
	
	input clk, reset;
	input [3:0]direction_control;
   output [3:0] direction;
	
	reg [3:0] direction, next_direction;
    
   localparam  right   = 4'b0001,
               up  	  = 4'b0100,
               left    = 4'b1000,
               down    = 4'b0010;
   // Next state logic aka ois_snakeBodyur state table

	always @(posedge clk)
	begin
		if((direction_control == 4'd9/*4'b1110*/)	&& (direction != left))
			next_direction <= right;
		else if((direction_control == 4'd6 /*4'b1011*/) && (direction != down))
			next_direction <= up;
		else if((direction_control == 4'd7 /*4'b0111*/) && (direction != right))
			next_direction <= left;
		else if((direction_control == 4'd8/*4'b1101*/) && (direction != up))
			next_direction <= down;
		else next_direction <= direction;
   end
	
	always @(posedge clk)
	begin
		if (~reset)
			direction <= right;
		else
			direction = next_direction;
	end
	
endmodule

// speed ctl//////////////////////////////////////////////////////////////////////////////////

// work !!!// 
module Speed_control(clk, size, move);
	input clk;
	input [6:0] size;
	output reg move;

	reg [23:0]q;
	
	//initiallize q
	//assign q = 28'b0;
	
	always @(posedge clk)
	begin
		if (q == (24'd4999999 - size*40000)) begin // we need 2 HZ frequency
			move <= 1;
			q <= 0;
			end
		else begin
			move <= 0;
			q <= q + 1'b1;
			end
	end
endmodule

//////////  hex_display   //////////////////////////////////////////////////////////////////
module hex_decoder(hex_digit, segments);
    input [3:0] hex_digit;
    output reg [6:0] segments;
   
    always @(*)
        case (hex_digit)
            4'h0: segments = 7'b100_0000;
            4'h1: segments = 7'b111_1001;
            4'h2: segments = 7'b010_0100;
            4'h3: segments = 7'b011_0000;
            4'h4: segments = 7'b001_1001;
            4'h5: segments = 7'b001_0010;
            4'h6: segments = 7'b000_0010;
            4'h7: segments = 7'b111_1000;
            4'h8: segments = 7'b000_0000;
            4'h9: segments = 7'b001_1000;
            4'hA: segments = 7'b000_1000;
            4'hB: segments = 7'b000_0011;
            4'hC: segments = 7'b100_0110;
            4'hD: segments = 7'b010_0001;
            4'hE: segments = 7'b000_0110;
            4'hF: segments = 7'b000_1110;   
            default: segments = 7'h7f;
        endcase
endmodule


// for debug // 
module hex_display_direction(direction, segements);
	input [3:0] direction;
	output reg [7:0]segements;
	
	always @(*)
	begin
		case (direction[3:0])
			4'b1000: segements = 7'b1011111; //left
			4'b0001: segements = 7'b1111101; //right
			4'b0010: segements = 7'b0111111; //down
			4'b0100: segements = 7'b1111110; //up
		endcase
	end
endmodule

// Module taken from Joe Armitage & co
module ps2_rx
	(
		input wire clk, reset, 
		input wire ps2d, ps2c, rx_en,    // ps2 data and clock inputs, receive enable input
		output reg rx_done_tick,         // ps2 receive done tick
		output wire [7:0] rx_data        // data received 
	);
	
	// FSMD state declaration
	localparam 
		idle = 1'b0,
		rx   = 1'b1;
		
	// internal signal declaration
	reg state_reg, state_next;          // FSMD state register
	reg [7:0] filter_reg;               // shift register filter for ps2c
	wire [7:0] filter_next;             // next state value of ps2c filter register
	reg f_val_reg;                      // reg for ps2c filter value, either 1 or 0
	wire f_val_next;                    // next state for ps2c filter value
	reg [3:0] n_reg, n_next;            // register to keep track of bit number 
	reg [10:0] d_reg, d_next;           // register to shift in rx data
	wire neg_edge;                      // negative edge of ps2c clock filter value
	
	// register for ps2c filter register and filter value
	always @(posedge clk, posedge reset)
		if (reset)
			begin
			filter_reg <= 0;
			f_val_reg  <= 0;
			end
		else
			begin
			filter_reg <= filter_next;
			f_val_reg  <= f_val_next;
			end

	// next state value of ps2c filter: right shift in current ps2c value to register
	assign filter_next = {ps2c, filter_reg[7:1]};
	
	// filter value next state, 1 if all bits are 1, 0 if all bits are 0, else no change
	assign f_val_next = (filter_reg == 8'b11111111) ? 1'b1 :
			    (filter_reg == 8'b00000000) ? 1'b0 :
			    f_val_reg;
	
	// negative edge of filter value: if current value is 1, and next state value is 0
	assign neg_edge = f_val_reg & ~f_val_next;
	
	// FSMD state, bit number, and data registers
	always @(posedge clk, posedge reset)
		if (reset)
			begin
			state_reg <= idle;
			n_reg <= 0;
			d_reg <= 0;
			end
		else
			begin
			state_reg <= state_next;
			n_reg <= n_next;
			d_reg <= d_next;
			end
	
	// FSMD next state logic
	always @*
		begin
		
		// defaults
		state_next = state_reg;
		rx_done_tick = 1'b0;
		n_next = n_reg;
		d_next = d_reg;
		
		case (state_reg)
			
			idle:
				if (neg_edge & rx_en)                 // start bit received
					begin
					n_next = 4'b1010;             // set bit count down to 10
					state_next = rx;              // go to rx state
					end
				
			rx:                                           // shift in 8 data, 1 parity, and 1 stop bit
				begin
				if (neg_edge)                         // if ps2c negative edge...
					begin
					d_next = {ps2d, d_reg[10:1]}; // sample ps2d, right shift into data register
					n_next = n_reg - 1;           // decrement bit count
					end
			
				if (n_reg==0)                         // after 10 bits shifted in, go to done state
                                        begin
					 rx_done_tick = 1'b1;         // assert dat received done tick
					 state_next = idle;           // go back to idle
					 end
				end
		endcase
		end
		
	assign rx_data = d_reg[8:1]; // output data bits 
endmodule

// Module taken from Joe Armitage & co
module keyboard

    (
	input wire clk, reset,
        input wire ps2d, ps2c,               // ps2 data and clock lines
        output wire [7:0] scan_code,         // scan_code received from keyboard to process
        output wire scan_code_ready,         // signal to outer control system to sample scan_code
        output wire letter_case_out          // output to determine if scan code is converted to lower or upper ascii code for a key
    );
	
    // constant declarations
    localparam  BREAK    = 8'hf0, // break code
                SHIFT1   = 8'h12, // first shift scan
                SHIFT2   = 8'h59, // second shift scan
                CAPS     = 8'h58; // caps lock

    // FSM symbolic states
    localparam [2:0] lowercase          = 3'b000, // idle, process lower case letters
                     ignore_break       = 3'b001, // ignore repeated scan code after break code -F0- reeived
                     shift              = 3'b010, // process uppercase letters for shift key held
                     ignore_shift_break = 3'b011, // check scan code after F0, either idle or go back to uppercase
		     capslock           = 3'b100, // process uppercase letter after capslock button pressed
		     ignore_caps_break  = 3'b101; // check scan code after F0, either ignore repeat, or decrement caps_num
                     
               
    // internal signal declarations
    reg [2:0] state_reg, state_next;           // FSM state register and next state logic
    wire [7:0] scan_out;                       // scan code received from keyboard
    reg got_code_tick;                         // asserted to write current scan code received to FIFO
    wire scan_done_tick;                       // asserted to signal that ps2_rx has received a scan code
    reg letter_case;                           // 0 for lower case, 1 for uppercase, outputed to use when converting scan code to ascii
    reg [7:0] shift_type_reg, shift_type_next; // register to hold scan code for either of the shift keys or caps lock
    reg [1:0] caps_num_reg, caps_num_next;     // keeps track of number of capslock scan codes received in capslock state (3 before going back to lowecase state)
   
    // instantiate ps2 receiver
    ps2_rx ps2_rx_unit (.clk(clk), .reset(reset), .rx_en(1'b1), .ps2d(ps2d), .ps2c(ps2c), .rx_done_tick(scan_done_tick), .rx_data(scan_out));
	
	// FSM stat, shift_type, caps_num register 
    always @(posedge clk, posedge reset)
        if (reset)
			begin
			state_reg      <= lowercase;
			shift_type_reg <= 0;
			caps_num_reg   <= 0;
			end
        else
			begin    
                        state_reg      <= state_next;
			shift_type_reg <= shift_type_next;
			caps_num_reg   <= caps_num_next;
			end
			
    //FSM next state logic
    always @*
        begin
       
        // defaults
        got_code_tick   = 1'b0;
	letter_case     = 1'b0;
	caps_num_next   = caps_num_reg;
        shift_type_next = shift_type_reg;
        state_next      = state_reg;
       
        case(state_reg)
			
	    // state to process lowercase key strokes, go to uppercase state to process shift/capslock
            lowercase:
                begin  
                if(scan_done_tick)                                                                    // if scan code received
		    begin
		    if(scan_out == SHIFT1 || scan_out == SHIFT2)                                      // if code is shift    
		        begin
			shift_type_next = scan_out;                                                   // record which shift key was pressed
			state_next = shift;                                                           // go to shift state
			end
					
		    else if(scan_out == CAPS)                                                         // if code is capslock
		        begin
			caps_num_next = 2'b11;                                                        // set caps_num to 3, num of capslock scan codes to receive before going back to lowecase
			state_next = capslock;                                                        // go to capslock state
			end

		    else if (scan_out == BREAK)                                                       // else if code is break code
			state_next = ignore_break;                                                    // go to ignore_break state
	 
		    else                                                                              // else if code is none of the above...            
			got_code_tick = 1'b1;                                                         // assert got_code_tick to write scan_out to FIFO
		    end	
                end
            
	    // state to ignore repeated scan code after break code FO received in lowercase state
            ignore_break:
                begin
                if(scan_done_tick)                                                                    // if scan code received, 
                    state_next = lowercase;                                                           // go back to lowercase state
                end
            
	    // state to process scan codes after shift received in lowercase state
            shift:
                begin
                letter_case = 1'b1;                                                                   // routed out to convert scan code to upper value for a key
               
                if(scan_done_tick)                                                                    // if scan code received,
			begin
			if(scan_out == BREAK)                                                             // if code is break code                                            
			    state_next = ignore_shift_break;                                              // go to ignore_shift_break state to ignore repeated scan code after F0

			else if(scan_out != SHIFT1 && scan_out != SHIFT2 && scan_out != CAPS)             // else if code is not shift/capslock
			    got_code_tick = 1'b1;                                                         // assert got_code_tick to write scan_out to FIFO
			end
		end
				
	     // state to ignore repeated scan code after break code F0 received in shift state 
	     ignore_shift_break:
	         begin
		 if(scan_done_tick)                                                                // if scan code received
		     begin
		     if(scan_out == shift_type_reg)                                                // if scan code is shift key initially pressed
		         state_next = lowercase;                                                   // shift/capslock key unpressed, go back to lowercase state
		     else                                                                          // else repeated scan code received, go back to uppercase state
			 state_next = shift;
		     end
		 end  
				
	     // state to process scan codes after capslock code received in lowecase state
	     capslock:
	         begin
		 letter_case = 1'b1;                                                               // routed out to convert scan code to upper value for a key
					
		 if(caps_num_reg == 0)                                                             // if capslock code received 3 times, 
		     state_next = lowercase;                                                   // go back to lowecase state
						
		 if(scan_done_tick)                                                                // if scan code received
		     begin 
		     if(scan_out == CAPS)                                                          // if code is capslock, 
		         caps_num_next = caps_num_reg - 1;                                         // decrement caps_num
						
		     else if(scan_out == BREAK)                                                    // else if code is break, go to ignore_caps_break state
			 state_next = ignore_caps_break;
						
		     else if(scan_out != SHIFT1 && scan_out != SHIFT2)                             // else if code isn't a shift key
			 got_code_tick = 1'b1;                                                     // assert got_code_tick to write scan_out to FIFO
		     end
		 end
				
		 // state to ignore repeated scan code after break code F0 received in capslock state 
		 ignore_caps_break:
		     begin
		     if(scan_done_tick)                                                                // if scan code received
		         begin
			 if(scan_out == CAPS)                                                          // if code is capslock
			     caps_num_next = caps_num_reg - 1;                                         // decrement caps_num
			 state_next = capslock;                                                        // return to capslock state
			 end
		     end
					
        endcase
        end
		
    // output, route letter_case to output to use during scan to ascii code conversion
    assign letter_case_out = letter_case; 
	
    // output, route got_code_tick to out control circuit to signal when to sample scan_out 
    assign scan_code_ready = got_code_tick;
	
    // route scan code data out
    assign scan_code = scan_out;
	
endmodule

// Module modified from key2ascii by Joe Armitage & co
module key2ascii
    (
        input wire letter_case,
        input wire [7:0] scan_code,
        output reg [3:0] direction_control
    );
    
always @*
  begin
  case(scan_code)
	/* Player movement*/
	8'h1c: direction_control = 4'd1;   // a
	8'h23: direction_control = 4'd2;   // d
	8'h1b: direction_control = 4'd3;   // s
	8'h1d: direction_control = 4'd4;   // w
	8'h29: direction_control = 4'd5;   // space
	/* Player movement alternates for possible player 2 implementation*/
	8'h75: direction_control = 4'd6;   // DC1: Up Arrow
	8'h6B: direction_control = 4'd7;   // DC2: Left Arrow
	8'h72: direction_control = 4'd8;   // DC3: Down Arrow
	8'h74: direction_control = 4'd9;   // DC4: Right Arrow
	default: direction_control = 4'd4; // default up
  endcase
  end
endmodule
