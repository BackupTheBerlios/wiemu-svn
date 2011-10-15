load build/wiemu.so wiemu

# Node: 0
# Runs: TinyOS Blink Application
set n0 [Node]
$n0 setFlash "Blink.bin"
$n0 setID 1
$n0 setLocation 10 10 0
set d0 [Debugger]
$d0 setFile "debug0.dat"
$d0 enableDisassembly
$n0 setDebugger $d0
set t0 [Thread]
$t0 setNode $n0
#$t0 setSteps 20

# Node: 1
# Runs: TinyOS CntToRfm Application
set n1 [Node]
$n1 setFlash "CntToRfm.bin"
$n1 setID 2
$n1 setLocation 50 50 0
set d1 [Debugger]
$d1 setFile "debug1.dat"
$d1 enableDisassembly
$n1 setDebugger $d1
set t1 [Thread]
$t1 setNode $n1
#$t1 setSteps 30

# Start all threads
Thread_start
