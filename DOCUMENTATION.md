[![CI](https://github.com/lf-lang/lingua-franca/actions/workflows/ci.yml/badge.svg)](https://github.com/lf-lang/lingua-franca/actions/workflows/ci.yml)
[![API docs](https://github.com/lf-lang/lingua-franca/actions/workflows/api-docs.yml/badge.svg)](https://github.com/lf-lang/lingua-franca/actions/workflows/api-docs.yml)

# lingua-franca Documentation

## Automatically Generated Doc Files

The code in lingua-franca is documented with Javadoc-style comments that are automatically processed and deployed when you push updates to the repo.  The latest docs can be found here:

- [lingua-franca docs](https://www.lf-lang.org/lingua-franca/)

## Building Doc Files Locally

To clone the repo and build the doc files locally, simply do this:

- Install [doxygen](https://www.doxygen.nl)

- Check out this repo and build the docs:
  - `git clone git@github.com:lf-lang/lingua-franca.git`
  - `cd lingua-franca/docs`
  - `doxygen Doxyfile.in`

### View Documentation Files

- Point your browser to the generated HTML page:
  - `open build/html/index.html`
