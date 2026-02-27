#!/bin/bash

changes() {
  git diff --name-only HEAD $(git merge-base HEAD origin/$1)
}

if changes $1 | grep "$2" | grep -q -v "^.*md\|txt$"; then
  echo "changed_$3=1" >> $GITHUB_OUTPUT
else
  echo "changed_$3=0" >> $GITHUB_OUTPUT
fi
