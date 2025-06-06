#!/bin/bash

if [[ $# -ne 1 ]] ; then
  echo "# Usage: $0 <input.csv>"
  exit -1
elif [[ ! -r $1 ]] ; then
  echo "# Error: input "${CSV}" is not a readable file"
  exit -1
fi

CSV="$1"
NAME="${CSV%.csv}"
LOG="${NAME}.log"
RUN="job_script.sh"

if [[ ! -x ${RUN} ]] ; then
  echo "# Error: run file "${RUN}" must be an executable"
  exit -1
fi

echo "# Input file: ${CSV}"
echo "# Run script: ${RUN}"
echo "# Log file: ${LOG}"

HEADER=$(head -1 "${CSV}")
parallel --verbose -P $(nproc) -a ${CSV} --resume-failed --ungroup --tag --joblog ${LOG} --header '.*\n' ./${RUN} ${HEADER}
exitcode=$?

if [[ $exitcode == 0 ]] ; then
  echo "# Result: all $(($(wc -l ${CSV} | awk '{print $1}') - 1)) job steps ran successfully"
else
  missing=$(parallel --dry-run -a ${CSV} --resume-failed --joblog ${LOG} --header : {} | wc -l)
  echo "# Result: some jobs failed, still need to run $missing job steps"
  exit $exitcode
fi
