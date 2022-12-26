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

ldy #0          ; use y as test counter

; initialize and test nmi vector
ldx #<nmi         
stx $fffa
ldx #>nmi         
stx $fffb

iny             ; Test 1
brk             ; break to nmi
.byt $aa         ; break flag
ret:

jmp exit        ; if we get here brk didnt work

nmi:
iny             ; Test 2


; All tests pass!
tya             ; store number of tests in X
tax
ldy #0          ; clear Y to indicate success

exit:
.byt jam

