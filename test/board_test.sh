#!/bin/bash -eu

# z80retroshield automated test suite
# Author: hanyazou@gmail.com

run_tests() {
    local examples=$ROOTDIR/examples

    test Hello $examples/hello '
       expect "Z80 configured; launching program.\r"
       expect "Hello\r"
       expect -re "Z80 processor stopped \[0-9\]* cycles executed.\r"
    '

    test "RAM test" $examples/ram-test '
       expect "Z80 configured; launching program.\r"
       expect "RAM \\\[ 21 08 00 7E 3C 77 18 FB 00 \\\]  counter value: 0\r"
       expect "RAM \\\[ 21 08 00 7E 3C 77 18 FB 01 \\\]  counter value: 1\r"
       expect "RAM \\\[ 21 08 00 7E 3C 77 18 FB 02 \\\]  counter value: 2\r"
       expect -re "Z80 processor stopped \[0-9\]* cycles executed.\r"
    '

    test --send-lf Monitor $examples/monitor '
       expect "Z80 configured; launching program.\r"
       expect ">"
       send "D0000\r"
       expect "0000 31 FB 00 3E 3E D3 01 21 FB 00 36 0A DB 01 77 FE\r"
       expect ">"
       send "I0200 3E 4F D3 01 3E 4B D3 01 3E 0A D3 01 C9\r"
       expect ">"
       send "D0200\r"
       expect "0200 3E 4F D3 01 3E 4B D3 01 3E 0A D3 01 C9 00 00 00\r"
       expect ">"
       send "C0200\r"
       expect "OK\r"
       expect ">"
    '

    test --send-lf "Upper Case conv" $examples/uc '
       expect "Z80 configured; launching program.\r"
       expect ">"
       send "Hello, world!\r"
       expect "HELLO, WORLD!\r"
       send "Bye!\r"
       expect "BYE!\r"
    '

    test "Run basic" $examples/basic '
       expect "Memory top? "
       send "0\r"
       expect {
           "Ok\r" { }
           "Memory top? " {
               send "\r"
               expect "Ok\r"
           }
       }
       
       send "10 A=0:FOR I=1 TO 100:A=A+1:NEXT:PRINT A\r"
       send "RUN\r"
       expect -re ".*100.*\r"
       expect "Ok\r"
       send "PRINT \"Hello, world!\", A\r"
       expect -re "Hello, world! .*100.*\r"
    '

    test "Speed test" $examples/speed_test '
       expect "Z80 configured to run endless stream of NOPs.\r"
       for {set i 1} {$i <= 10} {incr i 1} {
           expect -re "Cycles per second second:\[0-9\]*\r"
       }
    '
}

usage()
{
    echo "boad_test [options] [commands]"
    echo "command:"
    echo "        cleanup delete downloaded packages"
    echo "           list list all test names"
    echo "option:"
    echo "  --run <names> run specified test only (use ',' for multiple names)"
    echo "  --no-download skip downloading packages"
    echo "      --verbose enable verbose output"
    echo "         --help display this message"
}

main()
{
    setup

    local done=false
    while (( $# > 0 )); do
        case "$1" in
        cleanup)
            cleanup
            done=true
            ;;
        list)
            opt_list_targets=true
            echo test names:
            run_tests
            done=true
            ;;
        -t|--run)
            shift
            opt_target_tests+=( $( echo "$1" | tr '[:upper:],' '[:lower:] ' ) )
            opt_target_specified=true
            ;;
        --no-download)
            opt_no_download=true
            ;;
        -v|--verbose)
            opt_verbose=true
            ;;
        -h|--help)
            usage
            done=true
            ;;
        -*)
            echo unknown option "'"$1"'"
            usage
            exit 1
            ;;
        *)
            echo invalid argument "'"$1"'"
            usage
            exit 1
            ;;
        esac
        shift
    done

    if $done; then
        exit 0
    fi

    if ! $opt_no_download; then
        download
    fi

    TGT_FQBN=arduino:avr:mega
    prepair
    run_tests

    TGT_FQBN=adafruit:samd:adafruit_grandcentral_m4
    prepair
    run_tests

    echo
    printf "  executed %d tests, %d failures\n" $TTL_COUNT $TTL_FAILURES
    cat  .tmp.summary
    if [ "$TTL_COUNT" != 0 ] && [ "$TTL_FAILURES" == 0 ]; then
        highlight ".  Succeeded"
        exit 0
    else
        highlight ".  FAILED"
        exit 1
    fi
}

setup() {
    ROOTDIR=..
    opt_list_targets=false
    opt_target_tests=( )
    opt_target_specified=false
    opt_no_download=false
    opt_verbose=false

    TTL_COUNT=0
    TTL_FAILURES=0
    rm -f .tmp*
    echo -n > .tmp.summary
}

download() {
    arduino-cli core update-index
    arduino-cli core install arduino:avr@1.8.6
    arduino-cli core install adafruit:samd@1.7.11
}

prepair() {
    echo Using $( arduino-cli version )
    echo "      $( which arduino-cli )"

    ACLI_OPTS=""
    ACLI_OPTS+=" --config-file ./arduino-cli.yaml"
    if $opt_verbose; then
        ACLI_OPTS+=" --verbose"
    fi

    TGT_PORT=$( arduino-cli board list | grep -e $TGT_FQBN | awk '{ print $1 }' )
    if [ "$TGT_PORT" != "" ]; then
        echo "Board $TGT_FQBN is found at port $TGT_PORT"
    else
        echo "Board $TGT_FQBN is not found"
    fi
}

test() {
    local res=N/A
    local send_return_code=--send-cr

    while (( $# > 0 )); do
        case "$1" in
        --send-cr|--send-lf)
            send_return_code=$1
            shift
            ;;
        -*)
            echo unknown option "'"$1"'"
            return 1
            ;;
        *)
            break
            ;;
        esac
    done

    local name="$1"
    local sketch="$2"
    local expect="$3"

    if $opt_list_targets; then
        echo "    $( echo "$name" | tr '[:upper:] ' '[:lower:]_' )"
        return
    fi

    if ! check_test_target "$name"; then
        return
    fi

    echo arduino-cli compile $ACLI_OPTS --fqbn $TGT_FQBN $sketch
    arduino-cli compile $ACLI_OPTS --fqbn $TGT_FQBN $sketch

    if [ "$TGT_PORT" == "" ]; then
        _test "$name"
        return
    fi
    
    echo arduino-cli upload $ACLI_OPTS --fqbn $TGT_FQBN --port $TGT_PORT --verify $sketch
    arduino-cli upload $ACLI_OPTS --fqbn $TGT_FQBN --port $TGT_PORT --verify $sketch

    local mon_opts="$ACLI_OPTS --fqbn $TGT_FQBN --port $TGT_PORT --config baudrate=115200"
    echo '#!/usr/bin/expect
          set timeout 20
          expect_before {
              timeout { puts "Timeout detected"; exit 2 }
              eof     { puts "Unexpected EOD";   exit 1 }
          }
          '"spawn ./serial-monitor-wrapper $send_return_code " \
              "arduino-cli monitor $mon_opts $expect" > .tmp.exp
    if expect .tmp.exp; then
        res=pass
    else
        res=FAIL
        TTL_FAILURES=$(( TTL_FAILURES + 1 ))
    fi
    rm .tmp.exp

    TTL_COUNT=$(( TTL_COUNT + 1 ))

    local log="$( printf "  %s %-20s %s" $res "$name" $TGT_FQBN)"
    highlight ".$log"
    echo "$log" >> .tmp.summary
}

_test() {
    while (( $# > 0 )); do
        case "$1" in
        -*)
            shift
            ;;
        *)
            break
            ;;
        esac
    done

    local name="$1"
    if $opt_list_targets; then
        echo "    $( echo "$name" | tr '[:upper:] ' '[:lower:]_' )"
        return
    fi

    if ! check_test_target "$name"; then
        return
    fi
    local log="$( printf "  %s %-20s %s" skip "$name" $TGT_FQBN)"
    highlight ".$log"
    echo "$log" >> .tmp.summary
}

check_test_target()
{
    local name=$( echo "$1" | tr '[:upper:] ' '[:lower:]_' )
    local test_target=false

    if ! $opt_target_specified; then
        return 0
    fi
    
    for test in "${opt_target_tests[@]}"; do
        if [ "${name}" == "${test%h}" ]; then
            test_target=true
            break
        fi
    done
    if $test_target; then
        return 0
    fi
    return 1
}

cleanup() {
    rm -rf download
    rm -rf .tmp*
}

highlight() {
    echo ==============================
    echo "$*" | sed -e 's/^.//'
    echo ==============================
}

export PATH=/Applications/Arduino\ IDE.app/Contents/Resources/app/node_modules/arduino-ide-extension/build/:$PATH

main $*

