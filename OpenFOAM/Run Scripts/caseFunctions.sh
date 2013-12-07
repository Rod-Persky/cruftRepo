#!/bin/bash

# The calling script could be wanting to run in either parallel or a single
# threaded solve operation, all you need to do is add functions runParallel
# and runSingle respectively. It is possible that they are very similar, so
# the functions could just set-up the case and then go into a common section
# this is just the entry point which is 'required'. Further, Allrun and
# Allclean has been put into this one file - it isn't really needed to have
# two different files; this also simplifies things through just having less
# files to copy from case to case

function cleanCase {
    echo "Cleaning Case"
}

function setupRun {
    blockMesh
    setFields
    addSwirlAndRotation
}

function runParallel {
    echo "Running in parallel"
    echo "Got MPIRUN = $mpirun"
    setupRun
    decomposePar
    $mpirun buoyantPimpleFoam -parallel
}

function runSingle {
    echo "Running Single Threaded"
    setupRun
    buoyantPimpleFoam
}

function postProcessing {
    cd postProcessing
    sample -case .. -latestTime
    cd ..
}

# Lets, by default, run the single case.
cleanCase
runSingle