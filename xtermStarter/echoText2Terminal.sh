#!/usr/bin/expect -f

set arg1 [lindex $argv 0]

# Get a Bash shell
spawn -noecho bash

# Wait for a prompt
expect "$ "

# Type something
send $arg1

# Hand over control to the user
interact

exit

