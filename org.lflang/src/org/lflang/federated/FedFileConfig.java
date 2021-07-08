package org.lflang.federated;

import java.io.File;
import java.io.IOException;

import org.lflang.FileConfig;

public class FedFileConfig extends FileConfig {
    
    public FedFileConfig(FileConfig fileConfig, String federateName) throws IOException {
        super(fileConfig);
        
        this.srcGenPath = getSrcGenPath(this.srcGenBasePath, this.srcPkgPath,
                this.srcPath, name + File.separator + federateName);
    }   

}
