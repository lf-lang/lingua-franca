from distutils.core import setup, Extension
setup(name="LinguaFranca", version="1.0",
              ext_modules=[Extension("LinguaFranca", ["LinguaFranca.c"])])
