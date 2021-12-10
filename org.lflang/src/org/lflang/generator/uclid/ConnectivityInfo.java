package org.lflang;

public class ConnectivityInfo {
    
    public Object   upstream;
    public Object   downstream;
    public boolean  isConnection;
    public boolean  isPhysical;
    public long     delay;
    
    public ConnectivityInfo(
        Object  _upstream,
        Object  _downstream,
        boolean _isConnection, 
        boolean _isPhysical, 
        long    _delay
    ){
        upstream        =   _upstream;
        downstream      =   _downstream;
        isConnection    =   _isConnection;
        isPhysical      =   _isPhysical;
        delay           =   _delay;
    }
}