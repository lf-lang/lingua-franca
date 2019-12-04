'use strict';
var __extends = (this && this.__extends) || (function () {
    var extendStatics = function (d, b) {
        extendStatics = Object.setPrototypeOf ||
            ({ __proto__: [] } instanceof Array && function (d, b) { d.__proto__ = b; }) ||
            function (d, b) { for (var p in b) if (b.hasOwnProperty(p)) d[p] = b[p]; };
        return extendStatics(d, b);
    };
    return function (d, b) {
        extendStatics(d, b);
        function __() { this.constructor = d; }
        d.prototype = b === null ? Object.create(b) : (__.prototype = b.prototype, new __());
    };
})();
exports.__esModule = true;
var reactor_1 = require("../reactor");
var Hello = /** @class */ (function (_super) {
    __extends(Hello, _super);
    function Hello() {
        return _super !== null && _super.apply(this, arguments) || this;
    }
    /**
     * Print tick and schedule a1
     * @override
     */
    Hello.prototype.react = function () {
        console.log("Hello World");
    };
    return Hello;
}(reactor_1.Reaction));
var Minimal = /** @class */ (function (_super) {
    __extends(Minimal, _super);
    function Minimal(timeout, name) {
        var _this = _super.call(this, timeout, name) || this;
        _this.t = new reactor_1.Timer(0, 0);
        _this.r = new Hello(_this, [_this.t], 0);
        _this._reactions = [_this.r];
        return _this;
    }
    return Minimal;
}(reactor_1.App));
exports.Minimal = Minimal;
var minimal = new Minimal(null, "Minimal");
minimal._start(function () { return null; }, function () { return null; });
