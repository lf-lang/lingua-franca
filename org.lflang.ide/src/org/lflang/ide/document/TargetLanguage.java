package org.lflang.ide.document;

import org.lflang.Target;

public enum TargetLanguage {
	C("c", "C"),
	CPP("cpp", "Cpp"),
	PYTHON("py", "Python"),
	TYPESCRIPT("ts", "TypeScript");
	
	private final String extension;
	private final String canonicalForm;
	
	TargetLanguage(String extension, String canonicalForm) {
		this.extension = extension;
		this.canonicalForm = canonicalForm;
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
		for (TargetLanguage target : TargetLanguage.values()) {
			if (target.canonicalForm.equals(s)) {
				return target;
			}
		}
		return null;
	}

	@Override
	public String toString() {
		return canonicalForm;
	}
}
