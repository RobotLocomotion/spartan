export SPARTAN_SOURCE_DIR=/home/drc/spartan
export SPARTAN_BUILD_DIR=/home/drc/spartan
export SPARTAN_INSTALL_DIR=/home/drc/spartan/install

export LD_LIBRARY_PATH=$SPARTAN_INSTALL_DIR/lib:$LD_LIBRARY_PATH
export PYTHONPATH=$PYTHONPATH:$SPARTAN_INSTALL_DIR/lib/python2.7/site-packages:$SPARTAN_INSTALL_DIR/lib/python2.7/dist-packages
export PATH=$SPARTAN_INSTALL_DIR/bin:$SPARTAN_SOURCE_DIR/scripts/bin:$PATH

alias makedirector='make -C $SPARTAN_BUILD_DIR/director/src/director-build install'
alias makedrake='make -C $SPARTAN_BUILD_DIR/drake/drake install'

alias cds='cd $SPARTAN_SOURCE_DIR'
alias cdbuild='cd $SPARTAN_BUILD_DIR'
