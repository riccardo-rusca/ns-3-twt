# Debugging ns-3 code with LLDB - a tutorial

## References

1. https://developer.apple.com/library/archive/documentation/General/Conceptual/lldb-guide/chapters/C1-Introduction-to-Debugging.html
2. https://www.youtube.com/watch?v=3BkEOvI36Ds&ab_channel=ilinx
3. https://opensource.apple.com/source/lldb/lldb-159/www/tutorial.html

Update: 1 March 2022 - For some reason, LLDB does not work with ns3 natively. It throws an error that TERM variable is not set.
    But XCode works and LLDB works with it. Follow instructions on https://www.nsnam.org/docs/tutorial/html/getting-started.html
    Change the working directory so that output files are accessible to xcode. Go to Product -> Scheme -> Edit scheme
        Change working directory to ns3-dev dir (wherever ./ns3 script is)
        Else, there will be errors when writing to files using relative paths
## Running ns-3 under LLDB.

```
./waf --run <programName> --command-template="lldb %s"
```

1. LLDB has several shorthand commands (aliases) for longer command names. The shorthand names are used here. 
2. Type in 'help \<command name\>' in LLDB for help. Just 'help' will print general help with all aliases.
3. LLDB remembers the previous command executed. Pressing enter executes the previous command.
4. 'exit' exits LLDB.

### Running, advancing, and keeping track:

1. Execute ```run``` or ```r``` in LLDB to run the program. Set breakpoints and watchpoints before running.
2. 'n' advances the program.
3. To print a variable, use
    ```
    print <variableName>
    ```
4. To print all the variables at a point of execution:
    ```
    fr v
    ```
which is short for ``` frame variable ```


```frame variable <varName>``` prints a particular variable.

5. To see at what point in program the debugger is at:
    ```frame info```

This will print the line and column number along with current function call.

6. A backtrace - displays how the program reached this state. Excellent tool to figure out the chain of function calls.

    ```thread backtrace <int backtrace limit>```

7. ```gui``` enters the GUI mode

## Setting breakpoints

Command: 'breakpoint' or 'b'

Usage: (Before you run the program) 

1. Break at function bla::foo
```
b bla::foo
```

This will create breakpoints at all matches to above function call.

2. To set breakpoint at a line number in a file:
```
b filename.cc:123
```

3. ``` breakpoint list``` lists all the breakpoints

4. ```breakpoint delete -f```  deletes all breakpoints including dummy breakpoints.

## Expressions

The expressions command lets us execute expressions that can change variable values while the program is running.