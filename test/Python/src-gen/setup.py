from setuptools import setup, Extension
setup(name="LinguaFrancaComposition", version="1.0",
        ext_modules=[Extension("LinguaFrancaComposition", ["Composition.c"])])
