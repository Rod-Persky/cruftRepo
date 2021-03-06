#!/bin/bash
# 
# Source clean functions
. $WM_PROJECT_DIR/bin/tools/CleanFunctions

#set -x

cleanSampling ()
{
  rm -rf sets > /dev/null 2>&1
}

cleanPostProcessing ()
{
  rm -rf postProcessing/*.png > /dev/null 2>&1
  rm -rf postProcessing/log.sample > /dev/null 2>&1
  rm -rf postProcessing/log.foamLog > /dev/null 2>&1
  rm -rf postProcessing/log.gnuplotPlotResiduals > /dev/null 2>&1
  rm -rf postProcessing/log.gnuplotCompareAll > /dev/null 2>&1
  rm -rf postProcessing/logs > /dev/null 2>&1
}

cleanFilesFromM4 ()
{
  rm {constant/polyMesh/boundary,constant/polyMesh/blockMeshDict} > /dev/null 2>&1
}

cleanTimeZero ()
{
  rm -rf 0 > /dev/null 2>&1
}

# Cleaning up the case
cleanSampling
cleanPostProcessing
cleanCase
cleanFilesFromM4
cleanTimeZero

rm time_pbs_single > /dev/null 2>&1
rm log_pbs_single > /dev/null 2>&1
