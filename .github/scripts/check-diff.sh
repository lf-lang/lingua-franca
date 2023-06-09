#!/bin/bash

changes() {
  git diff --name-only HEAD $(git merge-base HEAD origin/master)
}

if changes | grep $1 | grep -q -v '^.*md\|txt$'; then
  echo "changed_$2=1" >> $GITHUB_OUTPUT
else
  echo "changed_$2=0" >> $GITHUB_OUTPUT
fi
