module.exports = function (api) {
    api.cache(true);

    const presets = [ 
            "@babel/typescript"
    ];

    const plugins = [
            "@babel/proposal-class-properties",
            "@babel/proposal-object-rest-spread",
            "@babel/plugin-proposal-optional-chaining",
            "@babel/plugin-transform-modules-commonjs"
    ];

    return {
        presets,
        plugins
    };
}
