#!/bin/bash

# MIT License
#
# Copyright (c) 2020 Rik Baehnemann, ASL, ETH Zurich, Switzerland
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# --- SET UP GIT ---

echo "Do you wish to set up git? [y/n]? [Y,n]"
read install_git
if [[ $install_git == "Y" || $install_git == "y" ]]; then
  # fancy bash coloring for git :)
  sh -c 'echo "export GIT_PS1_SHOWDIRTYSTATE=1" >> ~/.bashrc'
  echo "export PS1='\[\033[03;32m\]\u@\h\[\033[01;34m\]:\w\[\033[02;33m\]\$(__git_ps1)\[\033[01;34m\]\\$\[\033[00m\] '" >> ~/.bashrc
  echo "export PROMPT_COMMAND='echo -ne \"\033]0;\$LOGNAME@\${HOSTNAME}: \${PWD}\007\"'" >> ~/.bashrc

  git config --global credential.helper cache
  git config --global push.default simple
  git config --global color.ui true

  git config --list

  echo ""

  echo "*** setting up ssh ***"
  if [ ! -f ~/.ssh/id_rsa.pub ]; then
      echo "ssh key not found!"
      echo "generating one for you"
      echo "$ ssh-keygen"
      ssh-keygen
  else
      echo "ssh public key found!"
  fi
  echo

  echo "$ cat ~/.ssh/id_rsa.pub"
  cat ~/.ssh/id_rsa.pub

  echo "Waiting for you to set up the git keys. Press any key when ready..."
  read -p "Press any key..."
fi
