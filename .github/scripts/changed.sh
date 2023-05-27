changes() {
  git diff --name-only --diff-filter=AMDR --cached origin/master
}

if changes | grep -q $1; then
  echo "::set-output name=CHANGED_$2::1"
else
  echo "::set-output name=CHANGED_$2::0"
fi

