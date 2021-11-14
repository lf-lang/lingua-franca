package org.lflang;

public class ConnectivityInfo {
    
    public boolean isConnection;
    public boolean isPhysical;
    public long delay;
    
    public ConnectivityInfo(boolean c, boolean p, long d) {
        isConnection    = c;
        isPhysical      = p;
        delay           = d;
    }
}