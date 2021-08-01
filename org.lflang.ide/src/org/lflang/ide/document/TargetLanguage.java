package org.lflang.ide.document;

public enum TargetLanguage {
	C("c"), CPP("cpp"), PYTHON("py"), TYPESCRIPT("ts");
	
	private final String extension;
	
	TargetLanguage(String extension) {
		this.extension = extension;
	}
	
	/**
	 * Returns the extension that should be expected at the end of files of
	 * this target language.
	 * @return the extension that should be expected at the end of files of
	 *     this target language
	 */
	public String getExtension() {
		return extension;
	}
	
	/**
	 * Returns the target language denoted by s. Returns null if s is not the
	 * canonical representation of any target language, as given in valid
	 * Lingua Franca target declarations.
	 * @param s an arbitrary string
	 * @return the target language denoted by s, if any such target language
	 *     exists; null otherwise
	 */
	public static TargetLanguage fromString(String s) {
		switch (s) {
			case "C":
				return C;
			case "Cpp":
				return CPP;
			case "Python":
				return PYTHON;
			case "TypeScript":
				return TYPESCRIPT;
			default:
				return null;
		}
	}
}
