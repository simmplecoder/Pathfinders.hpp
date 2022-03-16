#!/bin/sh

set -e

usage="$(basename "$0") [-g fgdir] [-h] -t target

where:
    -h  show this help text
    -t  executable to run under perf
    -g  root directory of FlameGraph repository https://github.com/brendangregg/FlameGraph
"

fgroot="."
while getopts 'g:ht:' option; do
    case "$option" in
      h) echo "$usage"
         exit
         ;;
      t) if ! [ -x "$(command -v "$OPTARG")" ]; then
            echo "$OPTARG is not executable"
            exit 1
         fi
         target="$OPTARG"
         ;;
      g) fgroot="$OPTARG"
         ;;
      :) printf "missing argument for -%s\n" "$OPTARG" >&2
         echo "$usage" >&2
         exit 1
         ;;
     \?) printf "illegal option: -%s\n" "$OPTARG" >&2
         echo "$usage" >&2
         exit 1
         ;;
    esac
done
shift $((OPTIND - 1))

scpath="$fgroot/stackcollapse-perf.pl"
fgpath="$fgroot/flamegraph.pl"
if ! [ "$(command -v "$scpath")" ]; then
  printf "could not find stackcollapse-perf.pl\n"
  exit 1
fi

if ! [ "$(command -v "$fgpath")" ]; then
  printf "could not find flamegraph.pl\n"
  exit 1
fi


#filename=$(date +"%Y-%m-%d-%Hh-%Mm-%Ss").perf
filename=$(date -u +"%Y-%m-%dT%H:%M:%S%Z")
perf record -g --call-graph=dwarf -F 99 -- "$target" "$filename".json
perf script > "$filename".perf
$scpath "$filename".perf > "$filename".collapsed
$fgpath "$filename".collapsed > "$filename".svg
