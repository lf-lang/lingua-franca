package org.lflang;

import java.util.ResourceBundle;

/** Static class for managing strings. */
public class LocalStrings {
  public static final ResourceBundle res = ResourceBundle.getBundle("org.lflang.StringsBundle");
  public static final String VERSION = res.getString("VERSION");
}
