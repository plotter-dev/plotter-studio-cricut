; Test shapes: a 40 mm square and a 15 mm-radius circle.
; Units mm, origin at the sheet's bottom-left (the library default), Y up.
G21 G90
M5
G0 X20 Y200
M3
G1 X60 Y200 F1000
G1 X60 Y240
G1 X20 Y240
G1 X20 Y200
M5
G0 X100 Y220
M3
G2 X100 Y220 I15 J0
M5
M2
