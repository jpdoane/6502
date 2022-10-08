data=$0000
text=$00a0

*=data
d1: .byt $aa
d2: .byt $01
d3: .byt d4
.dsb 10
d4: .byt $0f
d5: .byt $ff

.dsb text - *   ; zero pad until text

adc #$1         ; A = 0+1   = 1
adc <d1         ; A = 1+aa  = ab
sec             ; set carry bit
sbc <d2         ; A = ab-1  = aa
inx             ; X = 0+1   = 1
jmp (ind_label)
brk             ; these will jam is jmp doesnt work
brk
label:
inx             ; X = 0+2   = 2
and (<d1,x)     ; A = aa&0f = 0a
asl             ; A = 14
lsr             ; A = 0a
sec             ; set carry bit
ror             ; A = 85
rol             ; A = 0a

sta <d4         ; write A=0a to [d4] (zpg)
inc <d4         ; [d4]+1 = 0b (zpg)
inc d4          ; [d4]+1 = 0c (abs)
ldx d4          ; read [d4]=0c (abs)
bit d4          ; NVZ = 0          

txs             ; s = 0c
inx             ; x = 0d
tsx             ; x = 0c
loop:           ; 
dex             ; x--
bne loop        ; loop until x==0

stx d5          ; d5 = 0
and $0          ; Z = 0
bit d5          ; Z = 1

php
pha
plp
pla


brk             ; halt

.dsb 10

ind_label:
.byt <label
.byt >label

.dsb $0100 - *   ; zero pad until end of page
