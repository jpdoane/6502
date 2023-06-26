data=$0000
text=$00a0
subrt=$00c0
jam=$02
C=%00000001   ;flag bits in status
Z=%00000010
I=%00000100
D=%00001000
B=%00010000
V=%01000000
N=%10000000

*=data
total: .byt $04
passed: .byt $00
.dsb text - *   ; zero pad until text

sei             ;disable interrupts (set interrupt disable flag)
cld             ;turn decimal mode off
ldx #$ff        ; initialize stack
txs    


; initialize and test nmi vector
ldx #<nmi         
stx $fffa
ldx #>nmi         
stx $fffb

; test 1
brk             ; break to nmi
.byt $aa         ; break flag
ret:
inc passed

jsr subrt
inc passed

jmp exit

.byt jam

nmi:
inc passed
rti

.byt jam

.dsb subrt - *   ; zero pad until subrt
;subrt:
inc passed
rts

.byt jam

; All tests pass!

exit:
.byt jam

