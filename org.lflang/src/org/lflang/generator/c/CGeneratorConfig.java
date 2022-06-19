package org.lflang.generator.c;

public class CGeneratorConfig{

    public final boolean CCppMode;
    public final int timerSize;
    public final boolean isMicroSeconds;
    public final boolean isArduino; //Special Case since we need a custom main method and custom support file.

    public CGeneratorConfig(boolean CCppMode, int timerSize, boolean isMicroSeconds, boolean isArduino){
        this.CCppMode = CCppMode;
        this.timerSize = timerSize;
        this.isMicroSeconds = isMicroSeconds;
        this.isArduino = isArduino;
    }

}