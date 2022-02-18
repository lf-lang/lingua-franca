{
  description = "build script for the lingua-franca alarm clock";

  inputs = {
    utils.url = "github:numtide/flake-utils";
  };

  outputs = inputs@{self, utils, nixpkgs, ...}: 
    utils.lib.eachDefaultSystem (system: let 
      pkgs = nixpkgs.legacyPackages.${system};
      in rec {
        packages.lf-alarm-clock = nixpkgs.legacyPackages.${system}.callPackage ./derivation.nix {};
      }
    );
}
