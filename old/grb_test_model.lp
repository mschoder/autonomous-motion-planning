\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
 [ - 4 x(0) * x(1) + 2 x(0) ^2 - 4 y(0) * y(1) + 2 y(0) ^2 + 4 x(1) ^2
   - 4 x(1) * x(2) + 4 y(1) ^2 - 4 y(1) * y(2) + 4 x(2) ^2 - 4 x(2) * x(3)
   + 4 y(2) ^2 - 4 y(2) * y(3) + 4 x(3) ^2 - 4 x(3) * x(4) + 4 y(3) ^2
   - 4 y(3) * y(4) + 4 x(4) ^2 - 4 x(4) * x(5) + 4 y(4) ^2 - 4 y(4) * y(5)
   + 4 x(5) ^2 - 4 x(5) * x(6) + 4 y(5) ^2 - 4 y(5) * y(6) + 4 x(6) ^2
   - 4 x(6) * x(7) + 4 y(6) ^2 - 4 y(6) * y(7) + 4 x(7) ^2 - 4 x(7) * x(8)
   + 4 y(7) ^2 - 4 y(7) * y(8) + 4 x(8) ^2 - 4 x(8) * x(9) + 4 y(8) ^2
   - 4 y(8) * y(9) + 4 x(9) ^2 - 4 x(9) * x(10) + 4 y(9) ^2
   - 4 y(9) * y(10) + 4 x(10) ^2 - 4 x(10) * x(11) + 4 y(10) ^2
   - 4 y(10) * y(11) + 4 x(11) ^2 - 4 x(11) * x(12) + 4 y(11) ^2
   - 4 y(11) * y(12) + 4 x(12) ^2 - 4 x(12) * x(13) + 4 y(12) ^2
   - 4 y(12) * y(13) + 4 x(13) ^2 - 4 x(13) * x(14) + 4 y(13) ^2
   - 4 y(13) * y(14) + 4 x(14) ^2 - 4 x(14) * x(15) + 4 y(14) ^2
   - 4 y(14) * y(15) + 4 x(15) ^2 - 4 x(15) * x(16) + 4 y(15) ^2
   - 4 y(15) * y(16) + 2 x(16) ^2 + 2 y(16) ^2 ] / 2 
Subject To
 R0: x(0) = 0
 R1: y(0) = 0
 R2: x(16) = 0
 R3: y(16) = 250
 R4: - 1.294130201655504 x(1) + y(1) - 1e+06 z(0,0,1) <= 66.98666134911194
 R5: - 1.294130201655504 x(2) + y(2) - 1e+06 z(0,0,2) <= 66.98666134911194
 R6: - 1.294130201655504 x(3) + y(3) - 1e+06 z(0,0,3) <= 66.98666134911194
 R7: - 1.294130201655504 x(4) + y(4) - 1e+06 z(0,0,4) <= 66.98666134911194
 R8: - 1.294130201655504 x(5) + y(5) - 1e+06 z(0,0,5) <= 66.98666134911194
 R9: - 1.294130201655504 x(6) + y(6) - 1e+06 z(0,0,6) <= 66.98666134911194
 R10: - 1.294130201655504 x(7) + y(7) - 1e+06 z(0,0,7)
   <= 66.98666134911194
 R11: - 1.294130201655504 x(8) + y(8) - 1e+06 z(0,0,8)
   <= 66.98666134911194
 R12: - 1.294130201655504 x(9) + y(9) - 1e+06 z(0,0,9)
   <= 66.98666134911194
 R13: - 1.294130201655504 x(10) + y(10) - 1e+06 z(0,0,10)
   <= 66.98666134911194
 R14: - 1.294130201655504 x(11) + y(11) - 1e+06 z(0,0,11)
   <= 66.98666134911194
 R15: - 1.294130201655504 x(12) + y(12) - 1e+06 z(0,0,12)
   <= 66.98666134911194
 R16: - 1.294130201655504 x(13) + y(13) - 1e+06 z(0,0,13)
   <= 66.98666134911194
 R17: - 1.294130201655504 x(14) + y(14) - 1e+06 z(0,0,14)
   <= 66.98666134911194
 R18: - 1.294130201655504 x(15) + y(15) - 1e+06 z(0,0,15)
   <= 66.98666134911194
 R19: - 1.294130201655504 x(16) + y(16) - 1e+06 z(0,0,16)
   <= 66.98666134911194
 R20: - 2.83003552517772 x(1) + y(1) - 1e+06 z(0,1,1) <= 55.57707894547629
 R21: - 2.83003552517772 x(2) + y(2) - 1e+06 z(0,1,2) <= 55.57707894547629
 R22: - 2.83003552517772 x(3) + y(3) - 1e+06 z(0,1,3) <= 55.57707894547629
 R23: - 2.83003552517772 x(4) + y(4) - 1e+06 z(0,1,4) <= 55.57707894547629
 R24: - 2.83003552517772 x(5) + y(5) - 1e+06 z(0,1,5) <= 55.57707894547629
 R25: - 2.83003552517772 x(6) + y(6) - 1e+06 z(0,1,6) <= 55.57707894547629
 R26: - 2.83003552517772 x(7) + y(7) - 1e+06 z(0,1,7) <= 55.57707894547629
 R27: - 2.83003552517772 x(8) + y(8) - 1e+06 z(0,1,8) <= 55.57707894547629
 R28: - 2.83003552517772 x(9) + y(9) - 1e+06 z(0,1,9) <= 55.57707894547629
 R29: - 2.83003552517772 x(10) + y(10) - 1e+06 z(0,1,10)
   <= 55.57707894547629
 R30: - 2.83003552517772 x(11) + y(11) - 1e+06 z(0,1,11)
   <= 55.57707894547629
 R31: - 2.83003552517772 x(12) + y(12) - 1e+06 z(0,1,12)
   <= 55.57707894547629
 R32: - 2.83003552517772 x(13) + y(13) - 1e+06 z(0,1,13)
   <= 55.57707894547629
 R33: - 2.83003552517772 x(14) + y(14) - 1e+06 z(0,1,14)
   <= 55.57707894547629
 R34: - 2.83003552517772 x(15) + y(15) - 1e+06 z(0,1,15)
   <= 55.57707894547629
 R35: - 2.83003552517772 x(16) + y(16) - 1e+06 z(0,1,16)
   <= 55.57707894547629
 R36: - 4.578123963425392 x(1) + y(1) - 1e+06 z(0,2,1)
   <= 20.26186067800823
 R37: - 4.578123963425392 x(2) + y(2) - 1e+06 z(0,2,2)
   <= 20.26186067800823
 R38: - 4.578123963425392 x(3) + y(3) - 1e+06 z(0,2,3)
   <= 20.26186067800823
 R39: - 4.578123963425392 x(4) + y(4) - 1e+06 z(0,2,4)
   <= 20.26186067800823
 R40: - 4.578123963425392 x(5) + y(5) - 1e+06 z(0,2,5)
   <= 20.26186067800823
 R41: - 4.578123963425392 x(6) + y(6) - 1e+06 z(0,2,6)
   <= 20.26186067800823
 R42: - 4.578123963425392 x(7) + y(7) - 1e+06 z(0,2,7)
   <= 20.26186067800823
 R43: - 4.578123963425392 x(8) + y(8) - 1e+06 z(0,2,8)
   <= 20.26186067800823
 R44: - 4.578123963425392 x(9) + y(9) - 1e+06 z(0,2,9)
   <= 20.26186067800823
 R45: - 4.578123963425392 x(10) + y(10) - 1e+06 z(0,2,10)
   <= 20.26186067800823
 R46: - 4.578123963425392 x(11) + y(11) - 1e+06 z(0,2,11)
   <= 20.26186067800823
 R47: - 4.578123963425392 x(12) + y(12) - 1e+06 z(0,2,12)
   <= 20.26186067800823
 R48: - 4.578123963425392 x(13) + y(13) - 1e+06 z(0,2,13)
   <= 20.26186067800823
 R49: - 4.578123963425392 x(14) + y(14) - 1e+06 z(0,2,14)
   <= 20.26186067800823
 R50: - 4.578123963425392 x(15) + y(15) - 1e+06 z(0,2,15)
   <= 20.26186067800823
 R51: - 4.578123963425392 x(16) + y(16) - 1e+06 z(0,2,16)
   <= 20.26186067800823
 R52: - 1.559561109430398 x(1) - y(1) - 1e+06 z(0,3,1)
   <= -180.8331204661984
 R53: - 1.559561109430398 x(2) - y(2) - 1e+06 z(0,3,2)
   <= -180.8331204661984
 R54: - 1.559561109430398 x(3) - y(3) - 1e+06 z(0,3,3)
   <= -180.8331204661984
 R55: - 1.559561109430398 x(4) - y(4) - 1e+06 z(0,3,4)
   <= -180.8331204661984
 R56: - 1.559561109430398 x(5) - y(5) - 1e+06 z(0,3,5)
   <= -180.8331204661984
 R57: - 1.559561109430398 x(6) - y(6) - 1e+06 z(0,3,6)
   <= -180.8331204661984
 R58: - 1.559561109430398 x(7) - y(7) - 1e+06 z(0,3,7)
   <= -180.8331204661984
 R59: - 1.559561109430398 x(8) - y(8) - 1e+06 z(0,3,8)
   <= -180.8331204661984
 R60: - 1.559561109430398 x(9) - y(9) - 1e+06 z(0,3,9)
   <= -180.8331204661984
 R61: - 1.559561109430398 x(10) - y(10) - 1e+06 z(0,3,10)
   <= -180.8331204661984
 R62: - 1.559561109430398 x(11) - y(11) - 1e+06 z(0,3,11)
   <= -180.8331204661984
 R63: - 1.559561109430398 x(12) - y(12) - 1e+06 z(0,3,12)
   <= -180.8331204661984
 R64: - 1.559561109430398 x(13) - y(13) - 1e+06 z(0,3,13)
   <= -180.8331204661984
 R65: - 1.559561109430398 x(14) - y(14) - 1e+06 z(0,3,14)
   <= -180.8331204661984
 R66: - 1.559561109430398 x(15) - y(15) - 1e+06 z(0,3,15)
   <= -180.8331204661984
 R67: - 1.559561109430398 x(16) - y(16) - 1e+06 z(0,3,16)
   <= -180.8331204661984
 R68: 4.886311726384738 x(1) + y(1) - 1e+06 z(0,4,1) <= 24.95109155692315
 R69: 4.886311726384738 x(2) + y(2) - 1e+06 z(0,4,2) <= 24.95109155692315
 R70: 4.886311726384738 x(3) + y(3) - 1e+06 z(0,4,3) <= 24.95109155692315
 R71: 4.886311726384738 x(4) + y(4) - 1e+06 z(0,4,4) <= 24.95109155692315
 R72: 4.886311726384738 x(5) + y(5) - 1e+06 z(0,4,5) <= 24.95109155692315
 R73: 4.886311726384738 x(6) + y(6) - 1e+06 z(0,4,6) <= 24.95109155692315
 R74: 4.886311726384738 x(7) + y(7) - 1e+06 z(0,4,7) <= 24.95109155692315
 R75: 4.886311726384738 x(8) + y(8) - 1e+06 z(0,4,8) <= 24.95109155692315
 R76: 4.886311726384738 x(9) + y(9) - 1e+06 z(0,4,9) <= 24.95109155692315
 R77: 4.886311726384738 x(10) + y(10) - 1e+06 z(0,4,10)
   <= 24.95109155692315
 R78: 4.886311726384738 x(11) + y(11) - 1e+06 z(0,4,11)
   <= 24.95109155692315
 R79: 4.886311726384738 x(12) + y(12) - 1e+06 z(0,4,12)
   <= 24.95109155692315
 R80: 4.886311726384738 x(13) + y(13) - 1e+06 z(0,4,13)
   <= 24.95109155692315
 R81: 4.886311726384738 x(14) + y(14) - 1e+06 z(0,4,14)
   <= 24.95109155692315
 R82: 4.886311726384738 x(15) + y(15) - 1e+06 z(0,4,15)
   <= 24.95109155692315
 R83: 4.886311726384738 x(16) + y(16) - 1e+06 z(0,4,16)
   <= 24.95109155692315
 R84: z(0,0,1) + z(0,1,1) + z(0,2,1) + z(0,3,1) + z(0,4,1) + z(0,5,1) <= 5
 R85: z(0,0,2) + z(0,1,2) + z(0,2,2) + z(0,3,2) + z(0,4,2) + z(0,5,2) <= 5
 R86: z(0,0,3) + z(0,1,3) + z(0,2,3) + z(0,3,3) + z(0,4,3) + z(0,5,3) <= 5
 R87: z(0,0,4) + z(0,1,4) + z(0,2,4) + z(0,3,4) + z(0,4,4) + z(0,5,4) <= 5
 R88: z(0,0,5) + z(0,1,5) + z(0,2,5) + z(0,3,5) + z(0,4,5) + z(0,5,5) <= 5
 R89: z(0,0,6) + z(0,1,6) + z(0,2,6) + z(0,3,6) + z(0,4,6) + z(0,5,6) <= 5
 R90: z(0,0,7) + z(0,1,7) + z(0,2,7) + z(0,3,7) + z(0,4,7) + z(0,5,7) <= 5
 R91: z(0,0,8) + z(0,1,8) + z(0,2,8) + z(0,3,8) + z(0,4,8) + z(0,5,8) <= 5
 R92: z(0,0,9) + z(0,1,9) + z(0,2,9) + z(0,3,9) + z(0,4,9) + z(0,5,9) <= 5
 R93: z(0,0,10) + z(0,1,10) + z(0,2,10) + z(0,3,10) + z(0,4,10) + z(0,5,10)
   <= 5
 R94: z(0,0,11) + z(0,1,11) + z(0,2,11) + z(0,3,11) + z(0,4,11) + z(0,5,11)
   <= 5
 R95: z(0,0,12) + z(0,1,12) + z(0,2,12) + z(0,3,12) + z(0,4,12) + z(0,5,12)
   <= 5
 R96: z(0,0,13) + z(0,1,13) + z(0,2,13) + z(0,3,13) + z(0,4,13) + z(0,5,13)
   <= 5
 R97: z(0,0,14) + z(0,1,14) + z(0,2,14) + z(0,3,14) + z(0,4,14) + z(0,5,14)
   <= 5
 R98: z(0,0,15) + z(0,1,15) + z(0,2,15) + z(0,3,15) + z(0,4,15) + z(0,5,15)
   <= 5
 R99: z(0,0,16) + z(0,1,16) + z(0,2,16) + z(0,3,16) + z(0,4,16) + z(0,5,16)
   <= 5
 qc0: [ x(0) ^2 - 2 x(0) * x(1) + y(0) ^2 - 2 y(0) * y(1) + x(1) ^2
   + y(1) ^2 ] <= 10000
 qc1: [ x(1) ^2 - 2 x(1) * x(2) + y(1) ^2 - 2 y(1) * y(2) + x(2) ^2
   + y(2) ^2 ] <= 10000
 qc2: [ x(2) ^2 - 2 x(2) * x(3) + y(2) ^2 - 2 y(2) * y(3) + x(3) ^2
   + y(3) ^2 ] <= 10000
 qc3: [ x(3) ^2 - 2 x(3) * x(4) + y(3) ^2 - 2 y(3) * y(4) + x(4) ^2
   + y(4) ^2 ] <= 10000
 qc4: [ x(4) ^2 - 2 x(4) * x(5) + y(4) ^2 - 2 y(4) * y(5) + x(5) ^2
   + y(5) ^2 ] <= 10000
 qc5: [ x(5) ^2 - 2 x(5) * x(6) + y(5) ^2 - 2 y(5) * y(6) + x(6) ^2
   + y(6) ^2 ] <= 10000
 qc6: [ x(6) ^2 - 2 x(6) * x(7) + y(6) ^2 - 2 y(6) * y(7) + x(7) ^2
   + y(7) ^2 ] <= 10000
 qc7: [ x(7) ^2 - 2 x(7) * x(8) + y(7) ^2 - 2 y(7) * y(8) + x(8) ^2
   + y(8) ^2 ] <= 10000
 qc8: [ x(8) ^2 - 2 x(8) * x(9) + y(8) ^2 - 2 y(8) * y(9) + x(9) ^2
   + y(9) ^2 ] <= 10000
 qc9: [ x(9) ^2 - 2 x(9) * x(10) + y(9) ^2 - 2 y(9) * y(10) + x(10) ^2
   + y(10) ^2 ] <= 10000
 qc10: [ x(10) ^2 - 2 x(10) * x(11) + y(10) ^2 - 2 y(10) * y(11) + x(11) ^2
   + y(11) ^2 ] <= 10000
 qc11: [ x(11) ^2 - 2 x(11) * x(12) + y(11) ^2 - 2 y(11) * y(12) + x(12) ^2
   + y(12) ^2 ] <= 10000
 qc12: [ x(12) ^2 - 2 x(12) * x(13) + y(12) ^2 - 2 y(12) * y(13) + x(13) ^2
   + y(13) ^2 ] <= 10000
 qc13: [ x(13) ^2 - 2 x(13) * x(14) + y(13) ^2 - 2 y(13) * y(14) + x(14) ^2
   + y(14) ^2 ] <= 10000
 qc14: [ x(14) ^2 - 2 x(14) * x(15) + y(14) ^2 - 2 y(14) * y(15) + x(15) ^2
   + y(15) ^2 ] <= 10000
 qc15: [ x(15) ^2 - 2 x(15) * x(16) + y(15) ^2 - 2 y(15) * y(16) + x(16) ^2
   + y(16) ^2 ] <= 10000
Bounds
 -550 <= x(0) <= 300
 -300 <= y(0) <= 550
 -550 <= x(1) <= 300
 -300 <= y(1) <= 550
 -550 <= x(2) <= 300
 -300 <= y(2) <= 550
 -550 <= x(3) <= 300
 -300 <= y(3) <= 550
 -550 <= x(4) <= 300
 -300 <= y(4) <= 550
 -550 <= x(5) <= 300
 -300 <= y(5) <= 550
 -550 <= x(6) <= 300
 -300 <= y(6) <= 550
 -550 <= x(7) <= 300
 -300 <= y(7) <= 550
 -550 <= x(8) <= 300
 -300 <= y(8) <= 550
 -550 <= x(9) <= 300
 -300 <= y(9) <= 550
 -550 <= x(10) <= 300
 -300 <= y(10) <= 550
 -550 <= x(11) <= 300
 -300 <= y(11) <= 550
 -550 <= x(12) <= 300
 -300 <= y(12) <= 550
 -550 <= x(13) <= 300
 -300 <= y(13) <= 550
 -550 <= x(14) <= 300
 -300 <= y(14) <= 550
 -550 <= x(15) <= 300
 -300 <= y(15) <= 550
 -550 <= x(16) <= 300
 -300 <= y(16) <= 550
Binaries
 z(0,0,1) z(0,0,2) z(0,0,3) z(0,0,4) z(0,0,5) z(0,0,6) z(0,0,7) z(0,0,8)
 z(0,0,9) z(0,0,10) z(0,0,11) z(0,0,12) z(0,0,13) z(0,0,14) z(0,0,15)
 z(0,0,16) z(0,1,1) z(0,1,2) z(0,1,3) z(0,1,4) z(0,1,5) z(0,1,6) z(0,1,7)
 z(0,1,8) z(0,1,9) z(0,1,10) z(0,1,11) z(0,1,12) z(0,1,13) z(0,1,14)
 z(0,1,15) z(0,1,16) z(0,2,1) z(0,2,2) z(0,2,3) z(0,2,4) z(0,2,5) z(0,2,6)
 z(0,2,7) z(0,2,8) z(0,2,9) z(0,2,10) z(0,2,11) z(0,2,12) z(0,2,13)
 z(0,2,14) z(0,2,15) z(0,2,16) z(0,3,1) z(0,3,2) z(0,3,3) z(0,3,4) z(0,3,5)
 z(0,3,6) z(0,3,7) z(0,3,8) z(0,3,9) z(0,3,10) z(0,3,11) z(0,3,12)
 z(0,3,13) z(0,3,14) z(0,3,15) z(0,3,16) z(0,4,1) z(0,4,2) z(0,4,3)
 z(0,4,4) z(0,4,5) z(0,4,6) z(0,4,7) z(0,4,8) z(0,4,9) z(0,4,10) z(0,4,11)
 z(0,4,12) z(0,4,13) z(0,4,14) z(0,4,15) z(0,4,16) z(0,5,1) z(0,5,2)
 z(0,5,3) z(0,5,4) z(0,5,5) z(0,5,6) z(0,5,7) z(0,5,8) z(0,5,9) z(0,5,10)
 z(0,5,11) z(0,5,12) z(0,5,13) z(0,5,14) z(0,5,15) z(0,5,16)
End
