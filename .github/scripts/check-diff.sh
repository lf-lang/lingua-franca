changes() {
  git diff --name-only --diff-filter=AMDR --cached origin/master
}

if changes | grep -q $1; then
  echo "CHANGED_$2=1" >> $GITHUB_OUTPUT
else
  echo "CHANGED_$2=0" >> $GITHUB_OUTPUT
fi
