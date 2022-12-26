data=$0000
text=$00a0
jam=$02
C=%00000001   ;flag bits in status
Z=%00000010
I=%00000100
D=%00001000
B=%00010000
V=%01000000
N=%10000000

*=data
d1: .byt $aa

.dsb text - *   ; zero pad until text

ldx #$ff        ; initialize stack
txs    
cli             ; clear int flag
ldy #0          ; use y as test counter

; A=0+1 == 1
iny             ; Test 1
lda #0          ; A=0
adc #1          ; A+=1
cmp #1          ; A==1
bne exit

; A=1-1 == 0
iny             ; Test 2
sec
sbc #1          ; A-=1
cmp #0          ; A==0
bne exit

; A=0-1 == -1
iny             ; Test 3
sec
sbc #1          ; A-=1
cmp #$ff        ; A==-1
bne exit

; A = -1+1 = 0+C
iny             ; Test 4
clc             
adc #1          ; A+=1
cmp #$00        ; A==0
bne exit
iny             ; Test 5
php
pla
cmp #C 
bne exit

; All tests pass!
tya             ; store number of tests in X
tax
ldy #0          ; clear Y to indicate success

exit:
.byt jam


