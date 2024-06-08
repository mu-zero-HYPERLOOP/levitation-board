#!/usr/bin/env sh

######################################################################
# @author      : karlsassie (karlsasssie@gmail.com)
# @file        : run_all
# @created     : Thursday Jun 06, 2024 21:31:25 CEST
#
# @description : 
######################################################################


canzero gen levitation_board1 src/canzero
make -C build
$TERM -e $PWD/build/levitation-board&

canzero gen levitation_board2 src/canzero
make -C build
$TERM -e $PWD/build/levitation-board&

canzero gen levitation_board3 src/canzero
make -C build
$TERM -e $PWD/build/levitation-board&



