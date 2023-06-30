;==================================================================
; (C) 2002  Bird Computer
; All rights reserved.
;
; 6502 Processor Test Routine
;
; 	The basic format is to test the things needed for further
; testing first, like branches and compare, then move onto other
; instructions.
;==================================================================

UART=$0f00
XMIT_FUL=$08		; the transmit buffer is full
data_ptr=$08

text = $1000
.dsb text  ; zero pad until text
*=text

; If the program gets here then we know at least the boot strap
; worked okay, which is in itself a good test of processor health.

	; jsr		putmsg
	; .asc "Testing Processor", 10, 0
	
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
; First thing to test is branches. If you can't branch reliably
; then the validity of the remaining tests are in question.
; Test branches and also simultaneously some other simple
; instructions.
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

	jsr		putmsg
	.asc "Testing Branching... ", 10, 0


	sei

	sec
	bcs		bcsok
	jsr		putmsg
	.asc "BCS:F", 10, 0
	
bcsok:
	clc
	bcc		bccok
	jsr		putmsg
	.asc "BCC:F", 10, 0
	
bccok:
	lda		#$00
	beq		beqok
	jsr		putmsg
	.asc "BEQ:F", 10, 0
	

beqok:
	lda		#$80
	bne		bneok
	jsr		putmsg
	.asc "BNE:F", 10, 0
	
bneok:
	ora		#$00
	bmi		bmiok
	jsr		putmsg
	.asc "BMI:F", 10, 0
	
bmiok:
	eor		#$80
	bpl		bplok
	jsr		putmsg
	.asc "BPL:F", 10, 0
	
bplok:
	lda		#$7f
	clc
	adc		#$10		; should give signed overflow
	bvs		bvsok
	jsr		putmsg
	.asc "BVS:F", 10, 0
	
bvsok:
	clv
	bvc		bvcok
	jsr		putmsg
	.asc "BVC:F", 10, 0
	
bvcok:

;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
; Compare Instructions
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

	jsr		putmsg
	.asc "Testing Compares... ", 10, 0


	lda		#27			; bit 7 = 0
	clc
	cmp		#27
	bcc		cmperr
	bne		cmperr
	bmi		cmperr
	lda		#$A1
	cmp		#20
	bpl		cmperr		; should be neg.
	sec
	lda		#10
	cmp		#20			; should be a borrow here
	bcs		cmperr
	clv
	lda		#$80		; -128 - 32 = -160 should overflow
	cmp		#$20		; compare doesn't affect overflow
	bvs		cmperr
	bvc		cmpok

cmperr:
	jsr		putmsg
	.asc "CMP:F", 10, 0
	

cmpok:
	ldx		#27
	clc
	cpx		#27
	bcc		cpxerr
	bne		cpxerr
	bmi		cpxerr
	ldx		#$A1
	cpx		#20
	bpl		cpxerr
	ldx		#10
	cpx		#20			; should be a borrow here
	bcs		cpxerr
	clv
	ldx		#$80		; -128 - 32 = -160 should overflow
	cpx		#$20		; but cpx shouldn't change overflow
	bvs		cpxerr		
	bvc		cpxok

cpxerr:
	jsr		putmsg
	.asc "CPX:F", 10, 0
	

cpxok:
	ldy		#27
	clc
	cpy		#27
	bcc		cpyerr
	bne		cpyerr
	bmi		cpyerr
	ldy		#$B0
	cpy		#20
	bpl		cpyerr
	ldy		#10
	cpy		#20			; should be a borrow here
	bcs		cpyerr
	clv
	ldy		#$80		; -128 - 32 = -160 should overflow
	cpy		#$20		; but cpy shouldn't change overflow
	bvs		cpyerr		
	bvc		cpyok

cpyerr:
	jsr		putmsg
	.asc "CPY:F", 10, 0
	

cpyok:


;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
; Load
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

	jsr		putmsg
	.asc "Testing Loads... ", 10, 0

; lda

	clc
	lda		#0
	bne		ldaerr
	bmi		ldaerr
	bcs		ldaerr
	lda		#$80
	beq		ldaerr
	bpl		ldaerr

	lda		#$00
	sta		$800
	bne		ldaerr
	bmi		ldaerr
	bcs		ldaerr
	
	lda		#$ff
	lda		$800
	bne		ldaerr
	bmi		ldaerr
	bcs		ldaerr
	
	cmp		#0
	bne		ldaerr
	
	sec
	lda		#$ff
	sta		$800
	beq		ldaerr
	bpl		ldaerr
	bcc		ldaerr
	
	lda		#0
	lda		$800
	beq		ldaerr
	bpl		ldaerr
	bcc		ldaerr
	
	cmp		#$ff
	beq		ldaok

ldaerr:
	jsr		putmsg
	.asc "LDA:F", 10, 0
	
ldaok:

; ldx

	clc
	lda		#$80		; z = 0, n = 1
	ldx		#0
	bcs		ldxerr
	bne		ldxerr
	bmi		ldxerr

	stx		$800
	bne		ldxerr
	bmi		ldxerr
	bcs		ldxerr
	
	ldx		#$ff
	ldx		$800
	bne		ldxerr
	bmi		ldxerr
	bcs		ldxerr
	
	cpx		#0
	bne		ldxerr
	
	sec
	ldx		#$ff
	stx		$800
	beq		ldxerr
	bpl		ldxerr
	bcc		ldxerr
	
	ldx		#0
	ldx		$800
	beq		ldxerr
	bpl		ldxerr
	bcc		ldxerr
	
	cpx		#$ff
	beq		ldxok

ldxerr:
	jsr		putmsg
	.asc "LDX:F", 10, 0
	

ldxok:

; ldy

	clc
	lda		#$80		; z = 0, n = 1
	ldy		#0
	bcs		ldyerr
	bne		ldyerr
	bmi		ldyerr

	sty		$800
	bne		ldyerr
	bmi		ldyerr
	bcs		ldyerr
	
	ldy		#$ff
	ldy		$800
	bne		ldyerr
	bmi		ldyerr
	bcs		ldyerr
	
	cpy		#0
	bne		ldyerr
	
	sec
	ldy		#$ff
	sty		$800
	beq		ldyerr
	bpl		ldyerr
	bcc		ldyerr
	
	ldy		#0
	ldy		$800
	beq		ldyerr
	bpl		ldyerr
	bcc		ldyerr
	
	cpy		#$ff
	beq		ldyok

ldyerr:
	jsr		putmsg
	.asc "LDY:F", 10, 0
	

ldyok:

;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
; Test register transfers
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
	jsr		putmsg
	.asc "Testing Transfers... ", 10, 0

; tax

	clc
	lda		#0
	ldx		#$80			; z = 0, n = 1
	tax
	bcs		taxerr
	bmi		taxerr
	bne		taxerr
	
	txa
	bne		taxerr

	lda		#$ff
	sec
	ldx		#0
	tax
	bcc		taxerr
	bpl		taxerr
	beq		taxerr
	
	txa
	cmp		#$ff
	beq		taxok

taxerr:
	jsr		putmsg
	.asc "TAX:F", 10, 0
	
taxok:

; tay

	clc
	lda		#0
	ldy		#$80			; z = 0, n = 1
	tay
	bcs		tayerr
	bmi		tayerr
	bne		tayerr
	
	tya
	bne		tayerr

	lda		#$ff
	sec
	ldy		#0
	tay
	bcc		tayerr
	bpl		tayerr
	beq		tayerr
	
	tya
	cmp		#$ff
	beq		tayok

tayerr:
	jsr		putmsg
	.asc "TAY:F", 10, 0
	
tayok:


; txs

	ldx		#15
	txs
	ldx		#87
	tsx
	cpx		#15
	beq		txsok
	jsr		putmsg
	.asc "TSX:F", 10, 0
	
txsok:
	ldx		#87
	txa
	cmp		#87
	beq		txaok
	jsr		putmsg
	.asc "TXA:F", 10, 0
	
txaok:
	tay
	cpy		#87
	beq		tayok1
	jsr		putmsg
	.asc "TAY:F", 10, 0
	
tayok1:
	tya
	beq		tyaerr
	bmi		tyaerr
	cmp		#87
	beq		tyaok
tyaerr:
	jsr		putmsg
	.asc "TYA:F", 10, 0
	
tyaok:


;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
; Increment / Decrement
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
	jsr		putmsg
	.asc "Testing Inc/Dec... ", 10, 0

	ldx		#$FE
	clc
	lda		#0
	inx
	bcs		inxerr
	beq		inxerr
	bpl		inxerr
	
	cpx		#$ff
	bne		inxerr
	
	sec
	lda		#$80
	inx
	bcc		inxerr
	bne		inxerr
	bmi		inxerr
	
	cpx		#0
	bne		inxerr
	
	clc
inxl1				; test loop
	inx
	bcs		inxerr
	bne		inxl1
	
	sec
inxl2:
	inx
	bcc		inxerr
	bne		inxl2
	
	beq		inxok
	
inxerr:
	jsr		putmsg
	.asc "INX:F", 10, 0
	

inxok:

;	dex

	ldx		#2
	clc
	lda		#0
	dex
	bcs		dexerr
	beq		dexerr
	bmi		dexerr
	
	cpx		#1
	bne		dexerr
	
	sec
	lda		#$80
	dex
	bcc		dexerr
	bne		dexerr
	bmi		dexerr
	
	cpx		#0
	bne		dexerr
	
	lda		#0
	dex
	beq		dexerr
	bpl		dexerr
	
	cpx		#$ff
	bne		dexerr
	
	clc
dexl1:
	dex
	bcs		dexerr
	bne		dexl1
	
	sec
dexl2:
	dex
	bcc		dexerr
	bne		dexl2
	
	beq		dexok
	
dexerr:
	jsr		putmsg
	.asc "DEX:F", 10, 0
	
	
dexok:


; iny

	ldy		#$FE
	clc
	adc		#0
	iny
	bcs		inyerr
	beq		inyerr
	bpl		inyerr
	
	cpy		#$ff
	bne		inyerr
	
	sec
	lda		#$80
	iny
	bcc		inyerr
	bne		inyerr
	bmi		inyerr
	
	cpy		#0
	bne		inyerr
	
	clc
inyl1				; test loop
	iny
	bcs		inyerr
	bne		inyl1
	
	sec
inyl2:
	iny
	bcc		inyerr
	bne		inyl2
	
	beq		inyok
	
inyerr:
	jsr		putmsg
	.asc "INY:F", 10, 0
	


;	dey

inyok:

	ldy		#2
	clc
	lda		#0
	dey
	bcs		deyerr
	beq		deyerr
	bmi		deyerr
	
	cpy		#1
	bne		deyerr
	
	sec
	lda		#$80
	dey
	bcc		deyerr
	bne		deyerr
	bmi		deyerr
	
	cpy		#0
	bne		deyerr
	
	lda		#0
	dey
	beq		deyerr
	bpl		deyerr
	
	cpy		#$ff
	bne		deyerr
	
	clc
deyl1:
	dey
	bcs		deyerr
	bne		deyl1
	
	sec
deyl2:
	dey
	bcc		deyerr
	bne		deyl2
	
	beq		deyok
	
deyerr:
	jsr		putmsg
	.asc "DEY:F", 10, 0
	
	
deyok:


;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
; Stores
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
	jsr		putmsg
	.asc "Testing Stores... ", 10, 0

; sta

	lda		#0
	clc
	sta		$00
	bne		staerr
	bmi		staerr
	bcs		staerr
	
	lda		$00
	bne		staerr
	
	lda		#$ff
	sec
	sta		$00
	beq		staerr
	bpl		staerr
	bcc		staerr
	bcs		staok

staerr:
	jsr		putmsg
	.asc "STA:F", 10, 0
	
staok:

; stx

	ldx		#0
	clc
	stx		$00
	bne		stxerr
	bmi		stxerr
	bcs		stxerr
	
	ldx		$00
	bne		stxerr
	
	ldx		#$ff
	sec
	stx		$00
	beq		stxerr
	bpl		stxerr
	bcc		stxerr
	bcs		stxok

stxerr:
	jsr		putmsg
	.asc "STX:F", 10, 0
	
stxok:

; sty

	ldy		#0
	clc
	sty		$00
	bne		styerr
	bmi		styerr
	bcs		styerr
	
	ldy		$00
	bne		styerr
	
	ldy		#$ff
	sec
	sty		$00
	beq		styerr
	bpl		styerr
	bcc		styerr
	bcs		styok

styerr:
	jsr		putmsg
	.asc "STY:F", 10, 0
	
styok:

;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
; Test addressing mode
; 	Note that addressing modes are handled independently of the
; actual operation performed by the processor. This means that
; if a mode works with one instruction, it should work properly
; with all instructions that use that mode.
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

	jsr		putmsg
	.asc "Testing Addr modes... ", 10, 0

	lda		#$AA
	eor		#$55
	cmp		#$ff
	beq		imm_ok
	jsr		putmsg
	.asc "IMM:F", 10, 0
	
imm_ok:
	lda		n55
	eor		nAA
	clc
	adc		#1
	beq		abs_ok
	jsr		putmsg
	.asc "ABS:F", 10, 0
	
abs_ok:
	ldx		#2
	lda		n55,x
	cmp		#12
	beq		absx_ok
	jsr		putmsg
	.asc "ABS,X:F", 10, 0
	
absx_ok:
	ldy		#3
	lda		n55,y
	cmp		#34
	beq		absy_ok
	jsr		putmsg
	.asc "ABS,Y:F", 10, 0
	
absy_ok:
	lda		#33
	sta		data_ptr
	ldx		#0
	ldx		data_ptr
	cpx		#33
	beq		zp_ok
	jsr		putmsg
	.asc "ZP:F", 10, 0
	
zp_ok:
	lda		#44
	sta		data_ptr+33
	lda		data_ptr,x
	cmp		#44
	beq		zpx_ok
	jsr		putmsg
	.asc "ZP,X:F", 10, 0
	
zpx_ok:
	lda		#$12
	sta		$201
	lda		#$01
	sta		data_ptr
	lda		#$2
	sta		data_ptr+1
	ldx		#5
	lda		(data_ptr-5,x)
	cmp		#$12
	beq		zpix_ok
	jsr		putmsg
	.asc "(ZP,X):F", 10, 0
	
zpix_ok:
	lda		#$fe
	sta		data_ptr
	lda		#$01
	sta		data_ptr+1
	ldy		#3
	lda		(data_ptr),y
	cmp		#$12
	beq		zpiy_ok
	jsr		putmsg
	.asc "(ZP),y:F", 10, 0
	
zpiy_ok:
	ldy		#15
	ldx		data_ptr-15,y
	cpx		#$fe
	beq		zpy_ok
	jsr		putmsg
	.asc "ZP,Y:F", 10, 0
	
zpy_ok:


;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
	ldy		#$ff
	iny
	cpy		#0	
	beq		iny_ok
	jsr		putmsg
	.asc "INY:F", 10, 0
	
iny_ok:
	ldy		#10
	dey
	cpy		#9
	beq		dey_ok
	jsr		putmsg
	.asc "DEY:F", 10, 0
	
dey_ok:
	ldx		#$80
	inx
	cpx		#$81
	beq		inx_ok
	jsr		putmsg
	.asc "INX:F", 10, 0
	
inx_ok:
	ldx		#$00
	dex
	cpx		#$ff
	beq		dex_ok
	jsr		putmsg
	.asc "DEX:F", 10, 0
	
dex_ok:


;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
; Shift ops
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
	jsr		putmsg
	.asc "Testing Shifts... ", 10, 0

; asl a

	lda		#$01
	asl		;a
	asl	    ;a
	asl		;a
	asl		;a
	asl		;a
	asl		;a
	asl		;a
	bpl		aslaerr
	beq		aslaerr
	bcs		aslaerr
	cmp		#$80
	beq		aslaok

aslaerr:
	jsr		putmsg
	.asc "ASLA:F", 10, 0
	
	
aslaok:

; lsr a

	lsr		;a
	lsr		;a
	lsr		;a
	lsr		;a
	lsr		;a
	lsr		;a
	lsr		;a
	bmi		lsraerr
	beq		lsraerr
	bcs		lsraerr
	bcc		lsraok

lsraerr:
	jsr		putmsg
	.asc "LSRA:F", 10, 0
	
	
lsraok:


; rol a

	clc
	lda		#$01
	rol		;a
	rol		;a
	rol		;a
	rol		;a
	rol		;a
	rol		;a
	rol		;a
	bpl		rolaerr
	beq		rolaerr
	bcs		rolaerr
	cmp		#$80		; this will set the carry !!!
	bne		rolaerr
	clc
	rol		;a
	bcc		rolaerr
	bne		rolaerr
	bmi		rolaerr
	rol		;a
	bcs		rolaerr
	bmi		rolaerr
	beq		rolaerr
	cmp		#1
	beq		rolaok

rolaerr:
	jsr		putmsg
	.asc "ROLA:F", 10, 0
	
	jmp		rolaok
	
rolaok:


; ror a

	clc
	lda		#$80
	ror		;a
	ror		;a
	ror		;a
	ror		;a
	ror		;a
	ror		;a
	ror		;a
	bmi		roraerr
	beq		roraerr
	bcs		roraerr
	cmp		#$01		; this will set the carry !!!
	bne		roraerr
	clc
	ror		;a
	bcc		roraerr
	bne		roraerr
	bmi		roraerr
	ror		;a
	bcs		roraerr
	bpl		roraerr
	beq		roraerr
	cmp		#$80
	beq		roraok
	bne		roraerr

roraerr:
	jsr		putmsg
	.asc "RORA:F", 10, 0
	
	
roraok:

;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

	jsr		putmsg
	.asc "Testing Stack... ", 10, 0


; pha / pla
	lda		#$ee
	pha
	lda		#00
	clc
	pla					; doesn't affect flags
;	bmi		plaerr      ; JPD this is wrong, pla should set NZ flags
;	bne		plaerr
	bpl		plaerr      ; 
	beq		plaerr      ; 

	bcs		plaerr
	bcc		plaok

plaerr:
	jsr		putmsg
	.asc "PLA:F", 10, 0
	

plaok:
	jsr		putmsg
	.asc "Done!", 10, 0
    brk;
	; jmp		($FFFC)		; go back to reset

;------------------------------------------------------------------
; Kind of a chicken and egg problem here. If there is something
; wrong with the processor, then this code likely won't execute.
;

; put message to screen
; tests pla,sta,ldy,inc,bne,ora,jmp,jmp(abs)

; putmsg:
; 	pla					; pop the return address off the stack
; 	sta		data_ptr	; to use as a pointer to the data
; 	pla
; 	sta		data_ptr+1

;     ; inc data pointer to first character
; 	inc		data_ptr
; 	bne		pm2
; 	inc		data_ptr+1

; pm2:
; 	ldy		#$01
; 	lda		(data_ptr),y
; 	inc		data_ptr
; 	bne		pm3
; 	inc		data_ptr+1
; pm3:
; 	ora		#0			; end of string ?
; 	beq		pm1
; 	jsr		putSer
; 	jmp		pm2

; pm1:						; must update the return address !
; 	; inc		data_ptr
; 	; bne		pm4
; 	; inc		data_ptr+1

; pm4:
; 	jmp		(data_ptr)


putmsg
	pla					; pop the return address off the stack
	sta		data_ptr	; to use as a pointer to the data
	pla
	sta		data_ptr+1
pm2
	ldy		#$01
	lda		(data_ptr),y
	inc		data_ptr
	bne		pm3
	inc		data_ptr+1
pm3
	ora		#0			; end of string ?
	beq		pm1
	jsr		putSer
	jmp		pm2
pm1						; must update the return address !
	inc		data_ptr
	bne		pm4
	inc		data_ptr+1
pm4
	jmp		(data_ptr)


; put character to serial port
; test and,bne,pha,pla,sta,lda,rts

putSer:
	pha					; temporarily save character
ps1:
	lda		UART+1		; get serial port status
	and		#XMIT_FUL	; is it full ?
	bne		ps1			; If full then wait
	pla					; get back the char to write
	sta		UART		; write it to the xmit register
	rts

n55:
	.byt		$55
nAA:
	.byt		$AA
	
	.byt		12
	.byt		34

.asc "We are at the end!", 10, 0