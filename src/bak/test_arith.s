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
total: .byt $05
passed: .byt $00

d1: .byt $aa

.dsb text - *   ; zero pad until text

ldx #$ff        ; initialize stack
txs    
cli             ; clear int flag

; test 1
; A=0+1 == 1
lda #0          ; A=0
adc #1          ; A+=1
cmp #1          ; A==1
bne exit
inc passed      ; test passed

; test 2
; A=1-1 == 0
sec
sbc #1          ; A-=1
cmp #0          ; A==0
bne exit
inc passed      ; test passed

; test 3
; A=0-1 == -1
sec
sbc #1          ; A-=1
cmp #$ff        ; A==-1
bne exit
inc passed      ; test passed

; test 4
; A = -1+1 = 0+C
clc             
adc #1          ; A+=1
cmp #$00        ; A==0
bne exit
inc passed      ; test passed


; Test 5
php
pla
cmp #C 
bne exit
inc passed      ; test passed

exit:
.byt jam


