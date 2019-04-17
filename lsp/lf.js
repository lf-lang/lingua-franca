(function() {
  'use strict';

  var extend = function (destination, source) {
    if (!destination || !source) return destination;
    for (var key in source) {
      if (destination[key] !== source[key])
        destination[key] = source[key];
    }
    return destination;
  };

  var formatError = function (input, offset, expected) {
    var lines = input.split(/\n/g),
        lineNo = 0,
        position = 0;

    while (position <= offset) {
      position += lines[lineNo].length + 1;
      lineNo += 1;
    }
    var message = 'Line ' + lineNo + ': expected ' + expected.join(', ') + '\n',
        line = lines[lineNo - 1];

    message += line + '\n';
    position -= line.length + 1;

    while (position < offset) {
      message += ' ';
      position += 1;
    }
    return message + '^';
  };

  var inherit = function (subclass, parent) {
    var chain = function() {};
    chain.prototype = parent.prototype;
    subclass.prototype = new chain();
    subclass.prototype.constructor = subclass;
  };

  var TreeNode = function(text, offset, elements) {
    this.text = text;
    this.offset = offset;
    this.elements = elements || [];
  };

  TreeNode.prototype.forEach = function(block, context) {
    for (var el = this.elements, i = 0, n = el.length; i < n; i++) {
      block.call(context, el[i], i, el);
    }
  };

  var TreeNode1 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['Spacing'] = elements[0];
  };
  inherit(TreeNode1, TreeNode);

  var TreeNode2 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['Spacing'] = elements[2];
    this['Assignment'] = elements[1];
  };
  inherit(TreeNode2, TreeNode);

  var TreeNode3 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['Spacing'] = elements[2];
    this['Declaration'] = elements[1];
  };
  inherit(TreeNode3, TreeNode);

  var TreeNode4 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['Spacing'] = elements[2];
    this['EmbeddedStatement'] = elements[1];
  };
  inherit(TreeNode4, TreeNode);

  var TreeNode5 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['PropertyDeclaration'] = elements[0];
    this['_'] = elements[1];
  };
  inherit(TreeNode5, TreeNode);

  var TreeNode6 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['AssignmentExpression'] = elements[0];
    this['_'] = elements[1];
  };
  inherit(TreeNode6, TreeNode);

  var TreeNode7 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['_'] = elements[1];
    this['id'] = elements[2];
  };
  inherit(TreeNode7, TreeNode);

  var TreeNode8 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['_'] = elements[1];
    this['id'] = elements[2];
  };
  inherit(TreeNode8, TreeNode);

  var TreeNode9 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['_'] = elements[1];
    this['id'] = elements[2];
  };
  inherit(TreeNode9, TreeNode);

  var TreeNode10 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['_'] = elements[2];
    this['Call'] = elements[3];
  };
  inherit(TreeNode10, TreeNode);

  var TreeNode11 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['id'] = elements[0];
    this['Arguments'] = elements[1];
  };
  inherit(TreeNode11, TreeNode);

  var TreeNode12 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['id'] = elements[4];
    this['_'] = elements[3];
  };
  inherit(TreeNode12, TreeNode);

  var TreeNode13 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['_'] = elements[3];
    this['Header'] = elements[2];
    this['Body'] = elements[4];
  };
  inherit(TreeNode13, TreeNode);

  var TreeNode14 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['_'] = elements[3];
    this['Header'] = elements[2];
    this['Body'] = elements[4];
  };
  inherit(TreeNode14, TreeNode);

  var TreeNode15 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['_'] = elements[1];
    this['EmbeddedStatement'] = elements[2];
  };
  inherit(TreeNode15, TreeNode);

  var TreeNode16 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['_'] = elements[1];
    this['EmbeddedStatement'] = elements[2];
  };
  inherit(TreeNode16, TreeNode);

  var TreeNode17 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['_'] = elements[1];
    this['EmbeddedStatement'] = elements[2];
  };
  inherit(TreeNode17, TreeNode);

  var TreeNode18 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['_'] = elements[3];
    this['Header'] = elements[2];
  };
  inherit(TreeNode18, TreeNode);

  var TreeNode19 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['id'] = elements[0];
    this['_'] = elements[1];
    this['DeclarationArgs'] = elements[2];
  };
  inherit(TreeNode19, TreeNode);

  var TreeNode20 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['_'] = elements[1];
    this['DeclarationParam'] = elements[2];
  };
  inherit(TreeNode20, TreeNode);

  var TreeNode21 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['_'] = elements[1];
    this['DeclarationParam'] = elements[2];
  };
  inherit(TreeNode21, TreeNode);

  var TreeNode22 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['_'] = elements[1];
    this['DeclarationParam'] = elements[2];
  };
  inherit(TreeNode22, TreeNode);

  var TreeNode23 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['_'] = elements[1];
    this['Header'] = elements[2];
  };
  inherit(TreeNode23, TreeNode);

  var TreeNode24 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['_'] = elements[1];
    this['DeclarationParam'] = elements[2];
  };
  inherit(TreeNode24, TreeNode);

  var TreeNode25 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['_'] = elements[1];
    this['DeclarationParam'] = elements[2];
  };
  inherit(TreeNode25, TreeNode);

  var TreeNode26 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['id'] = elements[0];
  };
  inherit(TreeNode26, TreeNode);

  var TreeNode27 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['Type'] = elements[1];
    this['_'] = elements[2];
    this['Expression'] = elements[4];
  };
  inherit(TreeNode27, TreeNode);

  var TreeNode28 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['Type'] = elements[1];
  };
  inherit(TreeNode28, TreeNode);

  var TreeNode29 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['DeclarationArgs'] = elements[1];
    this['Spacing'] = elements[3];
    this['EmbeddedStatement'] = elements[4];
  };
  inherit(TreeNode29, TreeNode);

  var TreeNode30 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['_'] = elements[2];
  };
  inherit(TreeNode30, TreeNode);

  var TreeNode31 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['_'] = elements[1];
    this['AssignmentExpression'] = elements[2];
  };
  inherit(TreeNode31, TreeNode);

  var TreeNode32 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['id'] = elements[0];
    this['_'] = elements[3];
    this['Expression'] = elements[4];
  };
  inherit(TreeNode32, TreeNode);

  var TreeNode33 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['_'] = elements[3];
    this['DeclarationParamList'] = elements[2];
  };
  inherit(TreeNode33, TreeNode);

  var TreeNode34 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['_'] = elements[3];
    this['ParamList'] = elements[2];
  };
  inherit(TreeNode34, TreeNode);

  var TreeNode35 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['DeclarationParam'] = elements[1];
  };
  inherit(TreeNode35, TreeNode);

  var TreeNode36 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['DeclarationParam'] = elements[0];
    this['_'] = elements[3];
  };
  inherit(TreeNode36, TreeNode);

  var TreeNode37 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['Param'] = elements[1];
  };
  inherit(TreeNode37, TreeNode);

  var TreeNode38 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['Param'] = elements[0];
    this['_'] = elements[3];
  };
  inherit(TreeNode38, TreeNode);

  var TreeNode39 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['id'] = elements[0];
  };
  inherit(TreeNode39, TreeNode);

  var TreeNode40 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['Type'] = elements[1];
    this['_'] = elements[2];
    this['Expression'] = elements[4];
  };
  inherit(TreeNode40, TreeNode);

  var TreeNode41 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['Type'] = elements[1];
  };
  inherit(TreeNode41, TreeNode);

  var TreeNode42 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['_'] = elements[3];
    this['Expression'] = elements[1];
  };
  inherit(TreeNode42, TreeNode);

  var TreeNode43 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['_'] = elements[2];
    this['Literal'] = elements[3];
  };
  inherit(TreeNode43, TreeNode);

  var TreeNode44 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['Spacing'] = elements[1];
  };
  inherit(TreeNode44, TreeNode);

  var TreeNode45 = function(text, offset, elements) {
    TreeNode.apply(this, arguments);
    this['EndOfLine'] = elements[2];
  };
  inherit(TreeNode45, TreeNode);

  var FAILURE = {};

  var Grammar = {
    _read_Grammar: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Grammar = this._cache._Grammar || {};
      var cached = this._cache._Grammar[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(2);
      var address1 = FAILURE;
      address1 = this._read_Spacing();
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        var remaining0 = 1, index2 = this._offset, elements1 = [], address3 = true;
        while (address3 !== FAILURE) {
          address3 = this._read_Stament();
          if (address3 !== FAILURE) {
            elements1.push(address3);
            --remaining0;
          }
        }
        if (remaining0 <= 0) {
          address2 = new TreeNode(this._input.substring(index2, this._offset), index2, elements1);
          this._offset = this._offset;
        } else {
          address2 = FAILURE;
        }
        if (address2 !== FAILURE) {
          elements0[1] = address2;
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode1(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._Grammar[index0] = [address0, this._offset];
      return address0;
    },

    _read_Stament: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Stament = this._cache._Stament || {};
      var cached = this._cache._Stament[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset;
      var index2 = this._offset, elements0 = new Array(3);
      var address1 = FAILURE;
      address1 = this._read_Spacing();
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        address2 = this._read_Assignment();
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address3 = FAILURE;
          address3 = this._read_Spacing();
          if (address3 !== FAILURE) {
            elements0[2] = address3;
          } else {
            elements0 = null;
            this._offset = index2;
          }
        } else {
          elements0 = null;
          this._offset = index2;
        }
      } else {
        elements0 = null;
        this._offset = index2;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode2(this._input.substring(index2, this._offset), index2, elements0);
        this._offset = this._offset;
      }
      if (address0 === FAILURE) {
        this._offset = index1;
        var index3 = this._offset, elements1 = new Array(3);
        var address4 = FAILURE;
        address4 = this._read_Spacing();
        if (address4 !== FAILURE) {
          elements1[0] = address4;
          var address5 = FAILURE;
          address5 = this._read_Declaration();
          if (address5 !== FAILURE) {
            elements1[1] = address5;
            var address6 = FAILURE;
            address6 = this._read_Spacing();
            if (address6 !== FAILURE) {
              elements1[2] = address6;
            } else {
              elements1 = null;
              this._offset = index3;
            }
          } else {
            elements1 = null;
            this._offset = index3;
          }
        } else {
          elements1 = null;
          this._offset = index3;
        }
        if (elements1 === null) {
          address0 = FAILURE;
        } else {
          address0 = new TreeNode3(this._input.substring(index3, this._offset), index3, elements1);
          this._offset = this._offset;
        }
        if (address0 === FAILURE) {
          this._offset = index1;
          var index4 = this._offset, elements2 = new Array(3);
          var address7 = FAILURE;
          address7 = this._read_Spacing();
          if (address7 !== FAILURE) {
            elements2[0] = address7;
            var address8 = FAILURE;
            address8 = this._read_EmbeddedStatement();
            if (address8 !== FAILURE) {
              elements2[1] = address8;
              var address9 = FAILURE;
              address9 = this._read_Spacing();
              if (address9 !== FAILURE) {
                elements2[2] = address9;
              } else {
                elements2 = null;
                this._offset = index4;
              }
            } else {
              elements2 = null;
              this._offset = index4;
            }
          } else {
            elements2 = null;
            this._offset = index4;
          }
          if (elements2 === null) {
            address0 = FAILURE;
          } else {
            address0 = new TreeNode4(this._input.substring(index4, this._offset), index4, elements2);
            this._offset = this._offset;
          }
          if (address0 === FAILURE) {
            this._offset = index1;
          }
        }
      }
      this._cache._Stament[index0] = [address0, this._offset];
      return address0;
    },

    _read_EmbeddedStatement: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._EmbeddedStatement = this._cache._EmbeddedStatement || {};
      var cached = this._cache._EmbeddedStatement[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(3);
      var address1 = FAILURE;
      var chunk0 = null;
      if (this._offset < this._inputSize) {
        chunk0 = this._input.substring(this._offset, this._offset + 2);
      }
      if (chunk0 === '{=') {
        address1 = new TreeNode(this._input.substring(this._offset, this._offset + 2), this._offset);
        this._offset = this._offset + 2;
      } else {
        address1 = FAILURE;
        if (this._offset > this._failure) {
          this._failure = this._offset;
          this._expected = [];
        }
        if (this._offset === this._failure) {
          this._expected.push('"{="');
        }
      }
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        var remaining0 = 0, index2 = this._offset, elements1 = [], address3 = true;
        while (address3 !== FAILURE) {
          var index3 = this._offset;
          var index4 = this._offset, elements2 = new Array(2);
          var address4 = FAILURE;
          var index5 = this._offset;
          var chunk1 = null;
          if (this._offset < this._inputSize) {
            chunk1 = this._input.substring(this._offset, this._offset + 1);
          }
          if (chunk1 === '=') {
            address4 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
            this._offset = this._offset + 1;
          } else {
            address4 = FAILURE;
            if (this._offset > this._failure) {
              this._failure = this._offset;
              this._expected = [];
            }
            if (this._offset === this._failure) {
              this._expected.push('"="');
            }
          }
          this._offset = index5;
          if (address4 === FAILURE) {
            address4 = new TreeNode(this._input.substring(this._offset, this._offset), this._offset);
            this._offset = this._offset;
          } else {
            address4 = FAILURE;
          }
          if (address4 !== FAILURE) {
            elements2[0] = address4;
            var address5 = FAILURE;
            if (this._offset < this._inputSize) {
              address5 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
              this._offset = this._offset + 1;
            } else {
              address5 = FAILURE;
              if (this._offset > this._failure) {
                this._failure = this._offset;
                this._expected = [];
              }
              if (this._offset === this._failure) {
                this._expected.push('<any char>');
              }
            }
            if (address5 !== FAILURE) {
              elements2[1] = address5;
            } else {
              elements2 = null;
              this._offset = index4;
            }
          } else {
            elements2 = null;
            this._offset = index4;
          }
          if (elements2 === null) {
            address3 = FAILURE;
          } else {
            address3 = new TreeNode(this._input.substring(index4, this._offset), index4, elements2);
            this._offset = this._offset;
          }
          if (address3 === FAILURE) {
            this._offset = index3;
            var index6 = this._offset, elements3 = new Array(3);
            var address6 = FAILURE;
            var chunk2 = null;
            if (this._offset < this._inputSize) {
              chunk2 = this._input.substring(this._offset, this._offset + 1);
            }
            if (chunk2 === '=') {
              address6 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
              this._offset = this._offset + 1;
            } else {
              address6 = FAILURE;
              if (this._offset > this._failure) {
                this._failure = this._offset;
                this._expected = [];
              }
              if (this._offset === this._failure) {
                this._expected.push('"="');
              }
            }
            if (address6 !== FAILURE) {
              elements3[0] = address6;
              var address7 = FAILURE;
              var index7 = this._offset;
              var chunk3 = null;
              if (this._offset < this._inputSize) {
                chunk3 = this._input.substring(this._offset, this._offset + 1);
              }
              if (chunk3 === '}') {
                address7 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
                this._offset = this._offset + 1;
              } else {
                address7 = FAILURE;
                if (this._offset > this._failure) {
                  this._failure = this._offset;
                  this._expected = [];
                }
                if (this._offset === this._failure) {
                  this._expected.push('"}"');
                }
              }
              this._offset = index7;
              if (address7 === FAILURE) {
                address7 = new TreeNode(this._input.substring(this._offset, this._offset), this._offset);
                this._offset = this._offset;
              } else {
                address7 = FAILURE;
              }
              if (address7 !== FAILURE) {
                elements3[1] = address7;
                var address8 = FAILURE;
                if (this._offset < this._inputSize) {
                  address8 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
                  this._offset = this._offset + 1;
                } else {
                  address8 = FAILURE;
                  if (this._offset > this._failure) {
                    this._failure = this._offset;
                    this._expected = [];
                  }
                  if (this._offset === this._failure) {
                    this._expected.push('<any char>');
                  }
                }
                if (address8 !== FAILURE) {
                  elements3[2] = address8;
                } else {
                  elements3 = null;
                  this._offset = index6;
                }
              } else {
                elements3 = null;
                this._offset = index6;
              }
            } else {
              elements3 = null;
              this._offset = index6;
            }
            if (elements3 === null) {
              address3 = FAILURE;
            } else {
              address3 = new TreeNode(this._input.substring(index6, this._offset), index6, elements3);
              this._offset = this._offset;
            }
            if (address3 === FAILURE) {
              this._offset = index3;
            }
          }
          if (address3 !== FAILURE) {
            elements1.push(address3);
            --remaining0;
          }
        }
        if (remaining0 <= 0) {
          address2 = new TreeNode(this._input.substring(index2, this._offset), index2, elements1);
          this._offset = this._offset;
        } else {
          address2 = FAILURE;
        }
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address9 = FAILURE;
          var chunk4 = null;
          if (this._offset < this._inputSize) {
            chunk4 = this._input.substring(this._offset, this._offset + 2);
          }
          if (chunk4 === '=}') {
            address9 = new TreeNode(this._input.substring(this._offset, this._offset + 2), this._offset);
            this._offset = this._offset + 2;
          } else {
            address9 = FAILURE;
            if (this._offset > this._failure) {
              this._failure = this._offset;
              this._expected = [];
            }
            if (this._offset === this._failure) {
              this._expected.push('"=}"');
            }
          }
          if (address9 !== FAILURE) {
            elements0[2] = address9;
          } else {
            elements0 = null;
            this._offset = index1;
          }
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._EmbeddedStatement[index0] = [address0, this._offset];
      return address0;
    },

    _read_Declaration: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Declaration = this._cache._Declaration || {};
      var cached = this._cache._Declaration[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset;
      address0 = this._read_EntityDeclaration();
      if (address0 === FAILURE) {
        this._offset = index1;
        var index2 = this._offset, elements0 = new Array(3);
        var address1 = FAILURE;
        address1 = this._read_PropertyDeclaration();
        if (address1 !== FAILURE) {
          elements0[0] = address1;
          var address2 = FAILURE;
          address2 = this._read__();
          if (address2 !== FAILURE) {
            elements0[1] = address2;
            var address3 = FAILURE;
            var chunk0 = null;
            if (this._offset < this._inputSize) {
              chunk0 = this._input.substring(this._offset, this._offset + 1);
            }
            if (chunk0 === ';') {
              address3 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
              this._offset = this._offset + 1;
            } else {
              address3 = FAILURE;
              if (this._offset > this._failure) {
                this._failure = this._offset;
                this._expected = [];
              }
              if (this._offset === this._failure) {
                this._expected.push('";"');
              }
            }
            if (address3 !== FAILURE) {
              elements0[2] = address3;
            } else {
              elements0 = null;
              this._offset = index2;
            }
          } else {
            elements0 = null;
            this._offset = index2;
          }
        } else {
          elements0 = null;
          this._offset = index2;
        }
        if (elements0 === null) {
          address0 = FAILURE;
        } else {
          address0 = new TreeNode5(this._input.substring(index2, this._offset), index2, elements0);
          this._offset = this._offset;
        }
        if (address0 === FAILURE) {
          this._offset = index1;
        }
      }
      this._cache._Declaration[index0] = [address0, this._offset];
      return address0;
    },

    _read_EntityDeclaration: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._EntityDeclaration = this._cache._EntityDeclaration || {};
      var cached = this._cache._EntityDeclaration[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset;
      address0 = this._read_Reactor();
      if (address0 === FAILURE) {
        this._offset = index1;
        address0 = this._read_Reaction();
        if (address0 === FAILURE) {
          this._offset = index1;
          address0 = this._read_Constructor();
          if (address0 === FAILURE) {
            this._offset = index1;
            address0 = this._read_Composite();
            if (address0 === FAILURE) {
              this._offset = index1;
              address0 = this._read_Preamble();
              if (address0 === FAILURE) {
                this._offset = index1;
                address0 = this._read_Initialize();
                if (address0 === FAILURE) {
                  this._offset = index1;
                }
              }
            }
          }
        }
      }
      this._cache._EntityDeclaration[index0] = [address0, this._offset];
      return address0;
    },

    _read_PropertyDeclaration: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._PropertyDeclaration = this._cache._PropertyDeclaration || {};
      var cached = this._cache._PropertyDeclaration[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset;
      address0 = this._read_Target();
      if (address0 === FAILURE) {
        this._offset = index1;
        address0 = this._read_Import();
        if (address0 === FAILURE) {
          this._offset = index1;
          address0 = this._read_Clock();
          if (address0 === FAILURE) {
            this._offset = index1;
            address0 = this._read_Port();
            if (address0 === FAILURE) {
              this._offset = index1;
              address0 = this._read_ReactorPrimitive();
              if (address0 === FAILURE) {
                this._offset = index1;
                address0 = this._read_Instance();
                if (address0 === FAILURE) {
                  this._offset = index1;
                  address0 = this._read_Language();
                  if (address0 === FAILURE) {
                    this._offset = index1;
                  }
                }
              }
            }
          }
        }
      }
      this._cache._PropertyDeclaration[index0] = [address0, this._offset];
      return address0;
    },

    _read_Assignment: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Assignment = this._cache._Assignment || {};
      var cached = this._cache._Assignment[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(3);
      var address1 = FAILURE;
      address1 = this._read_AssignmentExpression();
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        address2 = this._read__();
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address3 = FAILURE;
          var chunk0 = null;
          if (this._offset < this._inputSize) {
            chunk0 = this._input.substring(this._offset, this._offset + 1);
          }
          if (chunk0 === ';') {
            address3 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
            this._offset = this._offset + 1;
          } else {
            address3 = FAILURE;
            if (this._offset > this._failure) {
              this._failure = this._offset;
              this._expected = [];
            }
            if (this._offset === this._failure) {
              this._expected.push('";"');
            }
          }
          if (address3 !== FAILURE) {
            elements0[2] = address3;
          } else {
            elements0 = null;
            this._offset = index1;
          }
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode6(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._Assignment[index0] = [address0, this._offset];
      return address0;
    },

    _read_Expression: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Expression = this._cache._Expression || {};
      var cached = this._cache._Expression[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset;
      address0 = this._read_EmbeddedStatement();
      if (address0 === FAILURE) {
        this._offset = index1;
        address0 = this._read_New();
        if (address0 === FAILURE) {
          this._offset = index1;
          address0 = this._read_Call();
          if (address0 === FAILURE) {
            this._offset = index1;
            address0 = this._read_id();
            if (address0 === FAILURE) {
              this._offset = index1;
              address0 = this._read_Literal();
              if (address0 === FAILURE) {
                this._offset = index1;
              }
            }
          }
        }
      }
      this._cache._Expression[index0] = [address0, this._offset];
      return address0;
    },

    _read_Target: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Target = this._cache._Target || {};
      var cached = this._cache._Target[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(3);
      var address1 = FAILURE;
      var chunk0 = null;
      if (this._offset < this._inputSize) {
        chunk0 = this._input.substring(this._offset, this._offset + 6);
      }
      if (chunk0 === 'target') {
        address1 = new TreeNode(this._input.substring(this._offset, this._offset + 6), this._offset);
        this._offset = this._offset + 6;
      } else {
        address1 = FAILURE;
        if (this._offset > this._failure) {
          this._failure = this._offset;
          this._expected = [];
        }
        if (this._offset === this._failure) {
          this._expected.push('"target"');
        }
      }
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        address2 = this._read__();
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address3 = FAILURE;
          address3 = this._read_id();
          if (address3 !== FAILURE) {
            elements0[2] = address3;
          } else {
            elements0 = null;
            this._offset = index1;
          }
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode7(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._Target[index0] = [address0, this._offset];
      return address0;
    },

    _read_Language: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Language = this._cache._Language || {};
      var cached = this._cache._Language[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(3);
      var address1 = FAILURE;
      var chunk0 = null;
      if (this._offset < this._inputSize) {
        chunk0 = this._input.substring(this._offset, this._offset + 8);
      }
      if (chunk0 === 'language') {
        address1 = new TreeNode(this._input.substring(this._offset, this._offset + 8), this._offset);
        this._offset = this._offset + 8;
      } else {
        address1 = FAILURE;
        if (this._offset > this._failure) {
          this._failure = this._offset;
          this._expected = [];
        }
        if (this._offset === this._failure) {
          this._expected.push('"language"');
        }
      }
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        address2 = this._read__();
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address3 = FAILURE;
          address3 = this._read_id();
          if (address3 !== FAILURE) {
            elements0[2] = address3;
          } else {
            elements0 = null;
            this._offset = index1;
          }
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode8(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._Language[index0] = [address0, this._offset];
      return address0;
    },

    _read_Import: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Import = this._cache._Import || {};
      var cached = this._cache._Import[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(3);
      var address1 = FAILURE;
      var chunk0 = null;
      if (this._offset < this._inputSize) {
        chunk0 = this._input.substring(this._offset, this._offset + 6);
      }
      if (chunk0 === 'import') {
        address1 = new TreeNode(this._input.substring(this._offset, this._offset + 6), this._offset);
        this._offset = this._offset + 6;
      } else {
        address1 = FAILURE;
        if (this._offset > this._failure) {
          this._failure = this._offset;
          this._expected = [];
        }
        if (this._offset === this._failure) {
          this._expected.push('"import"');
        }
      }
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        address2 = this._read__();
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address3 = FAILURE;
          address3 = this._read_id();
          if (address3 !== FAILURE) {
            elements0[2] = address3;
          } else {
            elements0 = null;
            this._offset = index1;
          }
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode9(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._Import[index0] = [address0, this._offset];
      return address0;
    },

    _read_New: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._New = this._cache._New || {};
      var cached = this._cache._New[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(4);
      var address1 = FAILURE;
      address1 = this._read__();
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        var chunk0 = null;
        if (this._offset < this._inputSize) {
          chunk0 = this._input.substring(this._offset, this._offset + 3);
        }
        if (chunk0 === 'new') {
          address2 = new TreeNode(this._input.substring(this._offset, this._offset + 3), this._offset);
          this._offset = this._offset + 3;
        } else {
          address2 = FAILURE;
          if (this._offset > this._failure) {
            this._failure = this._offset;
            this._expected = [];
          }
          if (this._offset === this._failure) {
            this._expected.push('"new"');
          }
        }
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address3 = FAILURE;
          address3 = this._read__();
          if (address3 !== FAILURE) {
            elements0[2] = address3;
            var address4 = FAILURE;
            address4 = this._read_Call();
            if (address4 !== FAILURE) {
              elements0[3] = address4;
            } else {
              elements0 = null;
              this._offset = index1;
            }
          } else {
            elements0 = null;
            this._offset = index1;
          }
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode10(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._New[index0] = [address0, this._offset];
      return address0;
    },

    _read_Call: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Call = this._cache._Call || {};
      var cached = this._cache._Call[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(2);
      var address1 = FAILURE;
      address1 = this._read_id();
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        address2 = this._read_Arguments();
        if (address2 !== FAILURE) {
          elements0[1] = address2;
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode11(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._Call[index0] = [address0, this._offset];
      return address0;
    },

    _read_Port: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Port = this._cache._Port || {};
      var cached = this._cache._Port[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(5);
      var address1 = FAILURE;
      address1 = this._read_id();
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        address2 = this._read__();
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address3 = FAILURE;
          var chunk0 = null;
          if (this._offset < this._inputSize) {
            chunk0 = this._input.substring(this._offset, this._offset + 2);
          }
          if (chunk0 === '->') {
            address3 = new TreeNode(this._input.substring(this._offset, this._offset + 2), this._offset);
            this._offset = this._offset + 2;
          } else {
            address3 = FAILURE;
            if (this._offset > this._failure) {
              this._failure = this._offset;
              this._expected = [];
            }
            if (this._offset === this._failure) {
              this._expected.push('"->"');
            }
          }
          if (address3 !== FAILURE) {
            elements0[2] = address3;
            var address4 = FAILURE;
            address4 = this._read__();
            if (address4 !== FAILURE) {
              elements0[3] = address4;
              var address5 = FAILURE;
              address5 = this._read_id();
              if (address5 !== FAILURE) {
                elements0[4] = address5;
              } else {
                elements0 = null;
                this._offset = index1;
              }
            } else {
              elements0 = null;
              this._offset = index1;
            }
          } else {
            elements0 = null;
            this._offset = index1;
          }
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode12(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._Port[index0] = [address0, this._offset];
      return address0;
    },

    _read_Reactor: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Reactor = this._cache._Reactor || {};
      var cached = this._cache._Reactor[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(5);
      var address1 = FAILURE;
      var chunk0 = null;
      if (this._offset < this._inputSize) {
        chunk0 = this._input.substring(this._offset, this._offset + 7);
      }
      if (chunk0 === 'reactor') {
        address1 = new TreeNode(this._input.substring(this._offset, this._offset + 7), this._offset);
        this._offset = this._offset + 7;
      } else {
        address1 = FAILURE;
        if (this._offset > this._failure) {
          this._failure = this._offset;
          this._expected = [];
        }
        if (this._offset === this._failure) {
          this._expected.push('"reactor"');
        }
      }
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        address2 = this._read__();
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address3 = FAILURE;
          address3 = this._read_Header();
          if (address3 !== FAILURE) {
            elements0[2] = address3;
            var address4 = FAILURE;
            address4 = this._read__();
            if (address4 !== FAILURE) {
              elements0[3] = address4;
              var address5 = FAILURE;
              address5 = this._read_Body();
              if (address5 !== FAILURE) {
                elements0[4] = address5;
              } else {
                elements0 = null;
                this._offset = index1;
              }
            } else {
              elements0 = null;
              this._offset = index1;
            }
          } else {
            elements0 = null;
            this._offset = index1;
          }
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode13(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._Reactor[index0] = [address0, this._offset];
      return address0;
    },

    _read_Composite: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Composite = this._cache._Composite || {};
      var cached = this._cache._Composite[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(5);
      var address1 = FAILURE;
      var chunk0 = null;
      if (this._offset < this._inputSize) {
        chunk0 = this._input.substring(this._offset, this._offset + 9);
      }
      if (chunk0 === 'composite') {
        address1 = new TreeNode(this._input.substring(this._offset, this._offset + 9), this._offset);
        this._offset = this._offset + 9;
      } else {
        address1 = FAILURE;
        if (this._offset > this._failure) {
          this._failure = this._offset;
          this._expected = [];
        }
        if (this._offset === this._failure) {
          this._expected.push('"composite"');
        }
      }
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        address2 = this._read__();
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address3 = FAILURE;
          address3 = this._read_Header();
          if (address3 !== FAILURE) {
            elements0[2] = address3;
            var address4 = FAILURE;
            address4 = this._read__();
            if (address4 !== FAILURE) {
              elements0[3] = address4;
              var address5 = FAILURE;
              address5 = this._read_Body();
              if (address5 !== FAILURE) {
                elements0[4] = address5;
              } else {
                elements0 = null;
                this._offset = index1;
              }
            } else {
              elements0 = null;
              this._offset = index1;
            }
          } else {
            elements0 = null;
            this._offset = index1;
          }
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode14(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._Composite[index0] = [address0, this._offset];
      return address0;
    },

    _read_Constructor: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Constructor = this._cache._Constructor || {};
      var cached = this._cache._Constructor[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(3);
      var address1 = FAILURE;
      var chunk0 = null;
      if (this._offset < this._inputSize) {
        chunk0 = this._input.substring(this._offset, this._offset + 11);
      }
      if (chunk0 === 'constructor') {
        address1 = new TreeNode(this._input.substring(this._offset, this._offset + 11), this._offset);
        this._offset = this._offset + 11;
      } else {
        address1 = FAILURE;
        if (this._offset > this._failure) {
          this._failure = this._offset;
          this._expected = [];
        }
        if (this._offset === this._failure) {
          this._expected.push('"constructor"');
        }
      }
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        address2 = this._read__();
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address3 = FAILURE;
          address3 = this._read_EmbeddedStatement();
          if (address3 !== FAILURE) {
            elements0[2] = address3;
          } else {
            elements0 = null;
            this._offset = index1;
          }
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode15(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._Constructor[index0] = [address0, this._offset];
      return address0;
    },

    _read_Preamble: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Preamble = this._cache._Preamble || {};
      var cached = this._cache._Preamble[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(3);
      var address1 = FAILURE;
      var chunk0 = null;
      if (this._offset < this._inputSize) {
        chunk0 = this._input.substring(this._offset, this._offset + 8);
      }
      if (chunk0 === 'preamble') {
        address1 = new TreeNode(this._input.substring(this._offset, this._offset + 8), this._offset);
        this._offset = this._offset + 8;
      } else {
        address1 = FAILURE;
        if (this._offset > this._failure) {
          this._failure = this._offset;
          this._expected = [];
        }
        if (this._offset === this._failure) {
          this._expected.push('"preamble"');
        }
      }
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        address2 = this._read__();
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address3 = FAILURE;
          address3 = this._read_EmbeddedStatement();
          if (address3 !== FAILURE) {
            elements0[2] = address3;
          } else {
            elements0 = null;
            this._offset = index1;
          }
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode16(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._Preamble[index0] = [address0, this._offset];
      return address0;
    },

    _read_Initialize: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Initialize = this._cache._Initialize || {};
      var cached = this._cache._Initialize[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(3);
      var address1 = FAILURE;
      var chunk0 = null;
      if (this._offset < this._inputSize) {
        chunk0 = this._input.substring(this._offset, this._offset + 10);
      }
      if (chunk0 === 'initialize') {
        address1 = new TreeNode(this._input.substring(this._offset, this._offset + 10), this._offset);
        this._offset = this._offset + 10;
      } else {
        address1 = FAILURE;
        if (this._offset > this._failure) {
          this._failure = this._offset;
          this._expected = [];
        }
        if (this._offset === this._failure) {
          this._expected.push('"initialize"');
        }
      }
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        address2 = this._read__();
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address3 = FAILURE;
          address3 = this._read_EmbeddedStatement();
          if (address3 !== FAILURE) {
            elements0[2] = address3;
          } else {
            elements0 = null;
            this._offset = index1;
          }
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode17(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._Initialize[index0] = [address0, this._offset];
      return address0;
    },

    _read_Clock: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Clock = this._cache._Clock || {};
      var cached = this._cache._Clock[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(4);
      var address1 = FAILURE;
      var chunk0 = null;
      if (this._offset < this._inputSize) {
        chunk0 = this._input.substring(this._offset, this._offset + 5);
      }
      if (chunk0 === 'clock') {
        address1 = new TreeNode(this._input.substring(this._offset, this._offset + 5), this._offset);
        this._offset = this._offset + 5;
      } else {
        address1 = FAILURE;
        if (this._offset > this._failure) {
          this._failure = this._offset;
          this._expected = [];
        }
        if (this._offset === this._failure) {
          this._expected.push('"clock"');
        }
      }
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        address2 = this._read__();
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address3 = FAILURE;
          address3 = this._read_Header();
          if (address3 !== FAILURE) {
            elements0[2] = address3;
            var address4 = FAILURE;
            address4 = this._read__();
            if (address4 !== FAILURE) {
              elements0[3] = address4;
            } else {
              elements0 = null;
              this._offset = index1;
            }
          } else {
            elements0 = null;
            this._offset = index1;
          }
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode18(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._Clock[index0] = [address0, this._offset];
      return address0;
    },

    _read_Header: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Header = this._cache._Header || {};
      var cached = this._cache._Header[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset;
      var index2 = this._offset, elements0 = new Array(3);
      var address1 = FAILURE;
      address1 = this._read_id();
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        address2 = this._read__();
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address3 = FAILURE;
          address3 = this._read_DeclarationArgs();
          if (address3 !== FAILURE) {
            elements0[2] = address3;
          } else {
            elements0 = null;
            this._offset = index2;
          }
        } else {
          elements0 = null;
          this._offset = index2;
        }
      } else {
        elements0 = null;
        this._offset = index2;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode19(this._input.substring(index2, this._offset), index2, elements0);
        this._offset = this._offset;
      }
      if (address0 === FAILURE) {
        this._offset = index1;
        address0 = this._read_id();
        if (address0 === FAILURE) {
          this._offset = index1;
        }
      }
      this._cache._Header[index0] = [address0, this._offset];
      return address0;
    },

    _read_ReactorPrimitive: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._ReactorPrimitive = this._cache._ReactorPrimitive || {};
      var cached = this._cache._ReactorPrimitive[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset;
      address0 = this._read_Input();
      if (address0 === FAILURE) {
        this._offset = index1;
        address0 = this._read_Output();
        if (address0 === FAILURE) {
          this._offset = index1;
          address0 = this._read_Parameter();
          if (address0 === FAILURE) {
            this._offset = index1;
            address0 = this._read_Trigger();
            if (address0 === FAILURE) {
              this._offset = index1;
              address0 = this._read_Action();
              if (address0 === FAILURE) {
                this._offset = index1;
              }
            }
          }
        }
      }
      this._cache._ReactorPrimitive[index0] = [address0, this._offset];
      return address0;
    },

    _read_Input: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Input = this._cache._Input || {};
      var cached = this._cache._Input[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(3);
      var address1 = FAILURE;
      var chunk0 = null;
      if (this._offset < this._inputSize) {
        chunk0 = this._input.substring(this._offset, this._offset + 5);
      }
      if (chunk0 === 'input') {
        address1 = new TreeNode(this._input.substring(this._offset, this._offset + 5), this._offset);
        this._offset = this._offset + 5;
      } else {
        address1 = FAILURE;
        if (this._offset > this._failure) {
          this._failure = this._offset;
          this._expected = [];
        }
        if (this._offset === this._failure) {
          this._expected.push('\'input\'');
        }
      }
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        address2 = this._read__();
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address3 = FAILURE;
          address3 = this._read_DeclarationParam();
          if (address3 !== FAILURE) {
            elements0[2] = address3;
          } else {
            elements0 = null;
            this._offset = index1;
          }
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode20(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._Input[index0] = [address0, this._offset];
      return address0;
    },

    _read_Output: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Output = this._cache._Output || {};
      var cached = this._cache._Output[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(3);
      var address1 = FAILURE;
      var chunk0 = null;
      if (this._offset < this._inputSize) {
        chunk0 = this._input.substring(this._offset, this._offset + 6);
      }
      if (chunk0 === 'output') {
        address1 = new TreeNode(this._input.substring(this._offset, this._offset + 6), this._offset);
        this._offset = this._offset + 6;
      } else {
        address1 = FAILURE;
        if (this._offset > this._failure) {
          this._failure = this._offset;
          this._expected = [];
        }
        if (this._offset === this._failure) {
          this._expected.push('\'output\'');
        }
      }
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        address2 = this._read__();
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address3 = FAILURE;
          address3 = this._read_DeclarationParam();
          if (address3 !== FAILURE) {
            elements0[2] = address3;
          } else {
            elements0 = null;
            this._offset = index1;
          }
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode21(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._Output[index0] = [address0, this._offset];
      return address0;
    },

    _read_Parameter: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Parameter = this._cache._Parameter || {};
      var cached = this._cache._Parameter[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(3);
      var address1 = FAILURE;
      var chunk0 = null;
      if (this._offset < this._inputSize) {
        chunk0 = this._input.substring(this._offset, this._offset + 9);
      }
      if (chunk0 === 'parameter') {
        address1 = new TreeNode(this._input.substring(this._offset, this._offset + 9), this._offset);
        this._offset = this._offset + 9;
      } else {
        address1 = FAILURE;
        if (this._offset > this._failure) {
          this._failure = this._offset;
          this._expected = [];
        }
        if (this._offset === this._failure) {
          this._expected.push('\'parameter\'');
        }
      }
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        address2 = this._read__();
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address3 = FAILURE;
          address3 = this._read_DeclarationParam();
          if (address3 !== FAILURE) {
            elements0[2] = address3;
          } else {
            elements0 = null;
            this._offset = index1;
          }
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode22(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._Parameter[index0] = [address0, this._offset];
      return address0;
    },

    _read_Trigger: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Trigger = this._cache._Trigger || {};
      var cached = this._cache._Trigger[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset;
      var index2 = this._offset, elements0 = new Array(3);
      var address1 = FAILURE;
      var chunk0 = null;
      if (this._offset < this._inputSize) {
        chunk0 = this._input.substring(this._offset, this._offset + 7);
      }
      if (chunk0 === 'trigger') {
        address1 = new TreeNode(this._input.substring(this._offset, this._offset + 7), this._offset);
        this._offset = this._offset + 7;
      } else {
        address1 = FAILURE;
        if (this._offset > this._failure) {
          this._failure = this._offset;
          this._expected = [];
        }
        if (this._offset === this._failure) {
          this._expected.push('\'trigger\'');
        }
      }
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        address2 = this._read__();
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address3 = FAILURE;
          address3 = this._read_Header();
          if (address3 !== FAILURE) {
            elements0[2] = address3;
          } else {
            elements0 = null;
            this._offset = index2;
          }
        } else {
          elements0 = null;
          this._offset = index2;
        }
      } else {
        elements0 = null;
        this._offset = index2;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode23(this._input.substring(index2, this._offset), index2, elements0);
        this._offset = this._offset;
      }
      if (address0 === FAILURE) {
        this._offset = index1;
        var index3 = this._offset, elements1 = new Array(3);
        var address4 = FAILURE;
        var chunk1 = null;
        if (this._offset < this._inputSize) {
          chunk1 = this._input.substring(this._offset, this._offset + 7);
        }
        if (chunk1 === 'trigger') {
          address4 = new TreeNode(this._input.substring(this._offset, this._offset + 7), this._offset);
          this._offset = this._offset + 7;
        } else {
          address4 = FAILURE;
          if (this._offset > this._failure) {
            this._failure = this._offset;
            this._expected = [];
          }
          if (this._offset === this._failure) {
            this._expected.push('\'trigger\'');
          }
        }
        if (address4 !== FAILURE) {
          elements1[0] = address4;
          var address5 = FAILURE;
          address5 = this._read__();
          if (address5 !== FAILURE) {
            elements1[1] = address5;
            var address6 = FAILURE;
            address6 = this._read_DeclarationParam();
            if (address6 !== FAILURE) {
              elements1[2] = address6;
            } else {
              elements1 = null;
              this._offset = index3;
            }
          } else {
            elements1 = null;
            this._offset = index3;
          }
        } else {
          elements1 = null;
          this._offset = index3;
        }
        if (elements1 === null) {
          address0 = FAILURE;
        } else {
          address0 = new TreeNode24(this._input.substring(index3, this._offset), index3, elements1);
          this._offset = this._offset;
        }
        if (address0 === FAILURE) {
          this._offset = index1;
        }
      }
      this._cache._Trigger[index0] = [address0, this._offset];
      return address0;
    },

    _read_Action: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Action = this._cache._Action || {};
      var cached = this._cache._Action[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(3);
      var address1 = FAILURE;
      var chunk0 = null;
      if (this._offset < this._inputSize) {
        chunk0 = this._input.substring(this._offset, this._offset + 6);
      }
      if (chunk0 === 'action') {
        address1 = new TreeNode(this._input.substring(this._offset, this._offset + 6), this._offset);
        this._offset = this._offset + 6;
      } else {
        address1 = FAILURE;
        if (this._offset > this._failure) {
          this._failure = this._offset;
          this._expected = [];
        }
        if (this._offset === this._failure) {
          this._expected.push('\'action\'');
        }
      }
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        address2 = this._read__();
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address3 = FAILURE;
          address3 = this._read_DeclarationParam();
          if (address3 !== FAILURE) {
            elements0[2] = address3;
          } else {
            elements0 = null;
            this._offset = index1;
          }
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode25(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._Action[index0] = [address0, this._offset];
      return address0;
    },

    _read_DeclarationParam: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._DeclarationParam = this._cache._DeclarationParam || {};
      var cached = this._cache._DeclarationParam[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(2);
      var address1 = FAILURE;
      address1 = this._read_id();
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        var index2 = this._offset;
        var index3 = this._offset;
        var index4 = this._offset, elements1 = new Array(6);
        var address3 = FAILURE;
        var chunk0 = null;
        if (this._offset < this._inputSize) {
          chunk0 = this._input.substring(this._offset, this._offset + 1);
        }
        if (chunk0 === ':') {
          address3 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
          this._offset = this._offset + 1;
        } else {
          address3 = FAILURE;
          if (this._offset > this._failure) {
            this._failure = this._offset;
            this._expected = [];
          }
          if (this._offset === this._failure) {
            this._expected.push('":"');
          }
        }
        if (address3 !== FAILURE) {
          elements1[0] = address3;
          var address4 = FAILURE;
          address4 = this._read_Type();
          if (address4 !== FAILURE) {
            elements1[1] = address4;
            var address5 = FAILURE;
            address5 = this._read__();
            if (address5 !== FAILURE) {
              elements1[2] = address5;
              var address6 = FAILURE;
              var chunk1 = null;
              if (this._offset < this._inputSize) {
                chunk1 = this._input.substring(this._offset, this._offset + 1);
              }
              if (chunk1 === '(') {
                address6 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
                this._offset = this._offset + 1;
              } else {
                address6 = FAILURE;
                if (this._offset > this._failure) {
                  this._failure = this._offset;
                  this._expected = [];
                }
                if (this._offset === this._failure) {
                  this._expected.push('"("');
                }
              }
              if (address6 !== FAILURE) {
                elements1[3] = address6;
                var address7 = FAILURE;
                address7 = this._read_Expression();
                if (address7 !== FAILURE) {
                  elements1[4] = address7;
                  var address8 = FAILURE;
                  var chunk2 = null;
                  if (this._offset < this._inputSize) {
                    chunk2 = this._input.substring(this._offset, this._offset + 1);
                  }
                  if (chunk2 === ')') {
                    address8 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
                    this._offset = this._offset + 1;
                  } else {
                    address8 = FAILURE;
                    if (this._offset > this._failure) {
                      this._failure = this._offset;
                      this._expected = [];
                    }
                    if (this._offset === this._failure) {
                      this._expected.push('")"');
                    }
                  }
                  if (address8 !== FAILURE) {
                    elements1[5] = address8;
                  } else {
                    elements1 = null;
                    this._offset = index4;
                  }
                } else {
                  elements1 = null;
                  this._offset = index4;
                }
              } else {
                elements1 = null;
                this._offset = index4;
              }
            } else {
              elements1 = null;
              this._offset = index4;
            }
          } else {
            elements1 = null;
            this._offset = index4;
          }
        } else {
          elements1 = null;
          this._offset = index4;
        }
        if (elements1 === null) {
          address2 = FAILURE;
        } else {
          address2 = new TreeNode27(this._input.substring(index4, this._offset), index4, elements1);
          this._offset = this._offset;
        }
        if (address2 === FAILURE) {
          this._offset = index3;
          var index5 = this._offset, elements2 = new Array(2);
          var address9 = FAILURE;
          var chunk3 = null;
          if (this._offset < this._inputSize) {
            chunk3 = this._input.substring(this._offset, this._offset + 1);
          }
          if (chunk3 === ':') {
            address9 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
            this._offset = this._offset + 1;
          } else {
            address9 = FAILURE;
            if (this._offset > this._failure) {
              this._failure = this._offset;
              this._expected = [];
            }
            if (this._offset === this._failure) {
              this._expected.push('":"');
            }
          }
          if (address9 !== FAILURE) {
            elements2[0] = address9;
            var address10 = FAILURE;
            address10 = this._read_Type();
            if (address10 !== FAILURE) {
              elements2[1] = address10;
            } else {
              elements2 = null;
              this._offset = index5;
            }
          } else {
            elements2 = null;
            this._offset = index5;
          }
          if (elements2 === null) {
            address2 = FAILURE;
          } else {
            address2 = new TreeNode28(this._input.substring(index5, this._offset), index5, elements2);
            this._offset = this._offset;
          }
          if (address2 === FAILURE) {
            this._offset = index3;
          }
        }
        if (address2 === FAILURE) {
          address2 = new TreeNode(this._input.substring(index2, index2), index2);
          this._offset = index2;
        }
        if (address2 !== FAILURE) {
          elements0[1] = address2;
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode26(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._DeclarationParam[index0] = [address0, this._offset];
      return address0;
    },

    _read_Reaction: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Reaction = this._cache._Reaction || {};
      var cached = this._cache._Reaction[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(5);
      var address1 = FAILURE;
      var chunk0 = null;
      if (this._offset < this._inputSize) {
        chunk0 = this._input.substring(this._offset, this._offset + 8);
      }
      if (chunk0 === 'reaction') {
        address1 = new TreeNode(this._input.substring(this._offset, this._offset + 8), this._offset);
        this._offset = this._offset + 8;
      } else {
        address1 = FAILURE;
        if (this._offset > this._failure) {
          this._failure = this._offset;
          this._expected = [];
        }
        if (this._offset === this._failure) {
          this._expected.push('"reaction"');
        }
      }
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        address2 = this._read_DeclarationArgs();
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address3 = FAILURE;
          var index2 = this._offset;
          var index3 = this._offset, elements1 = new Array(4);
          var address4 = FAILURE;
          address4 = this._read__();
          if (address4 !== FAILURE) {
            elements1[0] = address4;
            var address5 = FAILURE;
            var chunk1 = null;
            if (this._offset < this._inputSize) {
              chunk1 = this._input.substring(this._offset, this._offset + 2);
            }
            if (chunk1 === '->') {
              address5 = new TreeNode(this._input.substring(this._offset, this._offset + 2), this._offset);
              this._offset = this._offset + 2;
            } else {
              address5 = FAILURE;
              if (this._offset > this._failure) {
                this._failure = this._offset;
                this._expected = [];
              }
              if (this._offset === this._failure) {
                this._expected.push('"->"');
              }
            }
            if (address5 !== FAILURE) {
              elements1[1] = address5;
              var address6 = FAILURE;
              address6 = this._read__();
              if (address6 !== FAILURE) {
                elements1[2] = address6;
                var address7 = FAILURE;
                var index4 = this._offset;
                address7 = this._read_DeclarationArgs();
                if (address7 === FAILURE) {
                  this._offset = index4;
                  address7 = this._read_id();
                  if (address7 === FAILURE) {
                    this._offset = index4;
                  }
                }
                if (address7 !== FAILURE) {
                  elements1[3] = address7;
                } else {
                  elements1 = null;
                  this._offset = index3;
                }
              } else {
                elements1 = null;
                this._offset = index3;
              }
            } else {
              elements1 = null;
              this._offset = index3;
            }
          } else {
            elements1 = null;
            this._offset = index3;
          }
          if (elements1 === null) {
            address3 = FAILURE;
          } else {
            address3 = new TreeNode30(this._input.substring(index3, this._offset), index3, elements1);
            this._offset = this._offset;
          }
          if (address3 === FAILURE) {
            address3 = new TreeNode(this._input.substring(index2, index2), index2);
            this._offset = index2;
          }
          if (address3 !== FAILURE) {
            elements0[2] = address3;
            var address8 = FAILURE;
            address8 = this._read_Spacing();
            if (address8 !== FAILURE) {
              elements0[3] = address8;
              var address9 = FAILURE;
              address9 = this._read_EmbeddedStatement();
              if (address9 !== FAILURE) {
                elements0[4] = address9;
              } else {
                elements0 = null;
                this._offset = index1;
              }
            } else {
              elements0 = null;
              this._offset = index1;
            }
          } else {
            elements0 = null;
            this._offset = index1;
          }
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode29(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._Reaction[index0] = [address0, this._offset];
      return address0;
    },

    _read_Instance: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Instance = this._cache._Instance || {};
      var cached = this._cache._Instance[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(3);
      var address1 = FAILURE;
      var chunk0 = null;
      if (this._offset < this._inputSize) {
        chunk0 = this._input.substring(this._offset, this._offset + 8);
      }
      if (chunk0 === 'instance') {
        address1 = new TreeNode(this._input.substring(this._offset, this._offset + 8), this._offset);
        this._offset = this._offset + 8;
      } else {
        address1 = FAILURE;
        if (this._offset > this._failure) {
          this._failure = this._offset;
          this._expected = [];
        }
        if (this._offset === this._failure) {
          this._expected.push('\'instance\'');
        }
      }
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        address2 = this._read__();
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address3 = FAILURE;
          address3 = this._read_AssignmentExpression();
          if (address3 !== FAILURE) {
            elements0[2] = address3;
          } else {
            elements0 = null;
            this._offset = index1;
          }
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode31(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._Instance[index0] = [address0, this._offset];
      return address0;
    },

    _read_AssignmentExpression: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._AssignmentExpression = this._cache._AssignmentExpression || {};
      var cached = this._cache._AssignmentExpression[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(5);
      var address1 = FAILURE;
      address1 = this._read_id();
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        address2 = this._read__();
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address3 = FAILURE;
          var chunk0 = null;
          if (this._offset < this._inputSize) {
            chunk0 = this._input.substring(this._offset, this._offset + 1);
          }
          if (chunk0 === '=') {
            address3 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
            this._offset = this._offset + 1;
          } else {
            address3 = FAILURE;
            if (this._offset > this._failure) {
              this._failure = this._offset;
              this._expected = [];
            }
            if (this._offset === this._failure) {
              this._expected.push('"="');
            }
          }
          if (address3 !== FAILURE) {
            elements0[2] = address3;
            var address4 = FAILURE;
            address4 = this._read__();
            if (address4 !== FAILURE) {
              elements0[3] = address4;
              var address5 = FAILURE;
              address5 = this._read_Expression();
              if (address5 !== FAILURE) {
                elements0[4] = address5;
              } else {
                elements0 = null;
                this._offset = index1;
              }
            } else {
              elements0 = null;
              this._offset = index1;
            }
          } else {
            elements0 = null;
            this._offset = index1;
          }
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode32(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._AssignmentExpression[index0] = [address0, this._offset];
      return address0;
    },

    _read_DeclarationArgs: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._DeclarationArgs = this._cache._DeclarationArgs || {};
      var cached = this._cache._DeclarationArgs[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(5);
      var address1 = FAILURE;
      var chunk0 = null;
      if (this._offset < this._inputSize) {
        chunk0 = this._input.substring(this._offset, this._offset + 1);
      }
      if (chunk0 === '(') {
        address1 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
        this._offset = this._offset + 1;
      } else {
        address1 = FAILURE;
        if (this._offset > this._failure) {
          this._failure = this._offset;
          this._expected = [];
        }
        if (this._offset === this._failure) {
          this._expected.push('"("');
        }
      }
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        address2 = this._read__();
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address3 = FAILURE;
          address3 = this._read_DeclarationParamList();
          if (address3 !== FAILURE) {
            elements0[2] = address3;
            var address4 = FAILURE;
            address4 = this._read__();
            if (address4 !== FAILURE) {
              elements0[3] = address4;
              var address5 = FAILURE;
              var chunk1 = null;
              if (this._offset < this._inputSize) {
                chunk1 = this._input.substring(this._offset, this._offset + 1);
              }
              if (chunk1 === ')') {
                address5 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
                this._offset = this._offset + 1;
              } else {
                address5 = FAILURE;
                if (this._offset > this._failure) {
                  this._failure = this._offset;
                  this._expected = [];
                }
                if (this._offset === this._failure) {
                  this._expected.push('")"');
                }
              }
              if (address5 !== FAILURE) {
                elements0[4] = address5;
              } else {
                elements0 = null;
                this._offset = index1;
              }
            } else {
              elements0 = null;
              this._offset = index1;
            }
          } else {
            elements0 = null;
            this._offset = index1;
          }
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode33(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._DeclarationArgs[index0] = [address0, this._offset];
      return address0;
    },

    _read_Arguments: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Arguments = this._cache._Arguments || {};
      var cached = this._cache._Arguments[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(5);
      var address1 = FAILURE;
      var chunk0 = null;
      if (this._offset < this._inputSize) {
        chunk0 = this._input.substring(this._offset, this._offset + 1);
      }
      if (chunk0 === '(') {
        address1 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
        this._offset = this._offset + 1;
      } else {
        address1 = FAILURE;
        if (this._offset > this._failure) {
          this._failure = this._offset;
          this._expected = [];
        }
        if (this._offset === this._failure) {
          this._expected.push('"("');
        }
      }
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        address2 = this._read__();
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address3 = FAILURE;
          address3 = this._read_ParamList();
          if (address3 !== FAILURE) {
            elements0[2] = address3;
            var address4 = FAILURE;
            address4 = this._read__();
            if (address4 !== FAILURE) {
              elements0[3] = address4;
              var address5 = FAILURE;
              var chunk1 = null;
              if (this._offset < this._inputSize) {
                chunk1 = this._input.substring(this._offset, this._offset + 1);
              }
              if (chunk1 === ')') {
                address5 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
                this._offset = this._offset + 1;
              } else {
                address5 = FAILURE;
                if (this._offset > this._failure) {
                  this._failure = this._offset;
                  this._expected = [];
                }
                if (this._offset === this._failure) {
                  this._expected.push('")"');
                }
              }
              if (address5 !== FAILURE) {
                elements0[4] = address5;
              } else {
                elements0 = null;
                this._offset = index1;
              }
            } else {
              elements0 = null;
              this._offset = index1;
            }
          } else {
            elements0 = null;
            this._offset = index1;
          }
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode34(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._Arguments[index0] = [address0, this._offset];
      return address0;
    },

    _read_DeclarationParamList: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._DeclarationParamList = this._cache._DeclarationParamList || {};
      var cached = this._cache._DeclarationParamList[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset;
      var index2 = this._offset, elements0 = new Array(2);
      var address1 = FAILURE;
      var remaining0 = 1, index3 = this._offset, elements1 = [], address2 = true;
      while (address2 !== FAILURE) {
        var index4 = this._offset, elements2 = new Array(4);
        var address3 = FAILURE;
        address3 = this._read_DeclarationParam();
        if (address3 !== FAILURE) {
          elements2[0] = address3;
          var address4 = FAILURE;
          address4 = this._read__();
          if (address4 !== FAILURE) {
            elements2[1] = address4;
            var address5 = FAILURE;
            var chunk0 = null;
            if (this._offset < this._inputSize) {
              chunk0 = this._input.substring(this._offset, this._offset + 1);
            }
            if (chunk0 === ',') {
              address5 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
              this._offset = this._offset + 1;
            } else {
              address5 = FAILURE;
              if (this._offset > this._failure) {
                this._failure = this._offset;
                this._expected = [];
              }
              if (this._offset === this._failure) {
                this._expected.push('","');
              }
            }
            if (address5 !== FAILURE) {
              elements2[2] = address5;
              var address6 = FAILURE;
              address6 = this._read__();
              if (address6 !== FAILURE) {
                elements2[3] = address6;
              } else {
                elements2 = null;
                this._offset = index4;
              }
            } else {
              elements2 = null;
              this._offset = index4;
            }
          } else {
            elements2 = null;
            this._offset = index4;
          }
        } else {
          elements2 = null;
          this._offset = index4;
        }
        if (elements2 === null) {
          address2 = FAILURE;
        } else {
          address2 = new TreeNode36(this._input.substring(index4, this._offset), index4, elements2);
          this._offset = this._offset;
        }
        if (address2 !== FAILURE) {
          elements1.push(address2);
          --remaining0;
        }
      }
      if (remaining0 <= 0) {
        address1 = new TreeNode(this._input.substring(index3, this._offset), index3, elements1);
        this._offset = this._offset;
      } else {
        address1 = FAILURE;
      }
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address7 = FAILURE;
        address7 = this._read_DeclarationParam();
        if (address7 !== FAILURE) {
          elements0[1] = address7;
        } else {
          elements0 = null;
          this._offset = index2;
        }
      } else {
        elements0 = null;
        this._offset = index2;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode35(this._input.substring(index2, this._offset), index2, elements0);
        this._offset = this._offset;
      }
      if (address0 === FAILURE) {
        this._offset = index1;
        var index5 = this._offset;
        address0 = this._read_DeclarationParam();
        if (address0 === FAILURE) {
          address0 = new TreeNode(this._input.substring(index5, index5), index5);
          this._offset = index5;
        }
        if (address0 === FAILURE) {
          this._offset = index1;
        }
      }
      this._cache._DeclarationParamList[index0] = [address0, this._offset];
      return address0;
    },

    _read_ParamList: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._ParamList = this._cache._ParamList || {};
      var cached = this._cache._ParamList[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset;
      var index2 = this._offset, elements0 = new Array(2);
      var address1 = FAILURE;
      var remaining0 = 1, index3 = this._offset, elements1 = [], address2 = true;
      while (address2 !== FAILURE) {
        var index4 = this._offset, elements2 = new Array(4);
        var address3 = FAILURE;
        address3 = this._read_Param();
        if (address3 !== FAILURE) {
          elements2[0] = address3;
          var address4 = FAILURE;
          address4 = this._read__();
          if (address4 !== FAILURE) {
            elements2[1] = address4;
            var address5 = FAILURE;
            var chunk0 = null;
            if (this._offset < this._inputSize) {
              chunk0 = this._input.substring(this._offset, this._offset + 1);
            }
            if (chunk0 === ',') {
              address5 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
              this._offset = this._offset + 1;
            } else {
              address5 = FAILURE;
              if (this._offset > this._failure) {
                this._failure = this._offset;
                this._expected = [];
              }
              if (this._offset === this._failure) {
                this._expected.push('","');
              }
            }
            if (address5 !== FAILURE) {
              elements2[2] = address5;
              var address6 = FAILURE;
              address6 = this._read__();
              if (address6 !== FAILURE) {
                elements2[3] = address6;
              } else {
                elements2 = null;
                this._offset = index4;
              }
            } else {
              elements2 = null;
              this._offset = index4;
            }
          } else {
            elements2 = null;
            this._offset = index4;
          }
        } else {
          elements2 = null;
          this._offset = index4;
        }
        if (elements2 === null) {
          address2 = FAILURE;
        } else {
          address2 = new TreeNode38(this._input.substring(index4, this._offset), index4, elements2);
          this._offset = this._offset;
        }
        if (address2 !== FAILURE) {
          elements1.push(address2);
          --remaining0;
        }
      }
      if (remaining0 <= 0) {
        address1 = new TreeNode(this._input.substring(index3, this._offset), index3, elements1);
        this._offset = this._offset;
      } else {
        address1 = FAILURE;
      }
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address7 = FAILURE;
        address7 = this._read_Param();
        if (address7 !== FAILURE) {
          elements0[1] = address7;
        } else {
          elements0 = null;
          this._offset = index2;
        }
      } else {
        elements0 = null;
        this._offset = index2;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode37(this._input.substring(index2, this._offset), index2, elements0);
        this._offset = this._offset;
      }
      if (address0 === FAILURE) {
        this._offset = index1;
        var index5 = this._offset;
        address0 = this._read_Param();
        if (address0 === FAILURE) {
          address0 = new TreeNode(this._input.substring(index5, index5), index5);
          this._offset = index5;
        }
        if (address0 === FAILURE) {
          this._offset = index1;
        }
      }
      this._cache._ParamList[index0] = [address0, this._offset];
      return address0;
    },

    _read_DeclarationParam: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._DeclarationParam = this._cache._DeclarationParam || {};
      var cached = this._cache._DeclarationParam[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(2);
      var address1 = FAILURE;
      address1 = this._read_id();
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        var index2 = this._offset;
        var index3 = this._offset;
        var index4 = this._offset, elements1 = new Array(6);
        var address3 = FAILURE;
        var chunk0 = null;
        if (this._offset < this._inputSize) {
          chunk0 = this._input.substring(this._offset, this._offset + 1);
        }
        if (chunk0 === ':') {
          address3 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
          this._offset = this._offset + 1;
        } else {
          address3 = FAILURE;
          if (this._offset > this._failure) {
            this._failure = this._offset;
            this._expected = [];
          }
          if (this._offset === this._failure) {
            this._expected.push('":"');
          }
        }
        if (address3 !== FAILURE) {
          elements1[0] = address3;
          var address4 = FAILURE;
          address4 = this._read_Type();
          if (address4 !== FAILURE) {
            elements1[1] = address4;
            var address5 = FAILURE;
            address5 = this._read__();
            if (address5 !== FAILURE) {
              elements1[2] = address5;
              var address6 = FAILURE;
              var chunk1 = null;
              if (this._offset < this._inputSize) {
                chunk1 = this._input.substring(this._offset, this._offset + 1);
              }
              if (chunk1 === '(') {
                address6 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
                this._offset = this._offset + 1;
              } else {
                address6 = FAILURE;
                if (this._offset > this._failure) {
                  this._failure = this._offset;
                  this._expected = [];
                }
                if (this._offset === this._failure) {
                  this._expected.push('"("');
                }
              }
              if (address6 !== FAILURE) {
                elements1[3] = address6;
                var address7 = FAILURE;
                address7 = this._read_Expression();
                if (address7 !== FAILURE) {
                  elements1[4] = address7;
                  var address8 = FAILURE;
                  var chunk2 = null;
                  if (this._offset < this._inputSize) {
                    chunk2 = this._input.substring(this._offset, this._offset + 1);
                  }
                  if (chunk2 === ')') {
                    address8 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
                    this._offset = this._offset + 1;
                  } else {
                    address8 = FAILURE;
                    if (this._offset > this._failure) {
                      this._failure = this._offset;
                      this._expected = [];
                    }
                    if (this._offset === this._failure) {
                      this._expected.push('")"');
                    }
                  }
                  if (address8 !== FAILURE) {
                    elements1[5] = address8;
                  } else {
                    elements1 = null;
                    this._offset = index4;
                  }
                } else {
                  elements1 = null;
                  this._offset = index4;
                }
              } else {
                elements1 = null;
                this._offset = index4;
              }
            } else {
              elements1 = null;
              this._offset = index4;
            }
          } else {
            elements1 = null;
            this._offset = index4;
          }
        } else {
          elements1 = null;
          this._offset = index4;
        }
        if (elements1 === null) {
          address2 = FAILURE;
        } else {
          address2 = new TreeNode40(this._input.substring(index4, this._offset), index4, elements1);
          this._offset = this._offset;
        }
        if (address2 === FAILURE) {
          this._offset = index3;
          var index5 = this._offset, elements2 = new Array(2);
          var address9 = FAILURE;
          var chunk3 = null;
          if (this._offset < this._inputSize) {
            chunk3 = this._input.substring(this._offset, this._offset + 1);
          }
          if (chunk3 === ':') {
            address9 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
            this._offset = this._offset + 1;
          } else {
            address9 = FAILURE;
            if (this._offset > this._failure) {
              this._failure = this._offset;
              this._expected = [];
            }
            if (this._offset === this._failure) {
              this._expected.push('":"');
            }
          }
          if (address9 !== FAILURE) {
            elements2[0] = address9;
            var address10 = FAILURE;
            address10 = this._read_Type();
            if (address10 !== FAILURE) {
              elements2[1] = address10;
            } else {
              elements2 = null;
              this._offset = index5;
            }
          } else {
            elements2 = null;
            this._offset = index5;
          }
          if (elements2 === null) {
            address2 = FAILURE;
          } else {
            address2 = new TreeNode41(this._input.substring(index5, this._offset), index5, elements2);
            this._offset = this._offset;
          }
          if (address2 === FAILURE) {
            this._offset = index3;
          }
        }
        if (address2 === FAILURE) {
          address2 = new TreeNode(this._input.substring(index2, index2), index2);
          this._offset = index2;
        }
        if (address2 !== FAILURE) {
          elements0[1] = address2;
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode39(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._DeclarationParam[index0] = [address0, this._offset];
      return address0;
    },

    _read_Param: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Param = this._cache._Param || {};
      var cached = this._cache._Param[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(4);
      var address1 = FAILURE;
      address1 = this._read__();
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        address2 = this._read_Expression();
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address3 = FAILURE;
          var index2 = this._offset;
          var index3 = this._offset, elements1 = new Array(4);
          var address4 = FAILURE;
          address4 = this._read__();
          if (address4 !== FAILURE) {
            elements1[0] = address4;
            var address5 = FAILURE;
            var chunk0 = null;
            if (this._offset < this._inputSize) {
              chunk0 = this._input.substring(this._offset, this._offset + 1);
            }
            if (chunk0 === '=') {
              address5 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
              this._offset = this._offset + 1;
            } else {
              address5 = FAILURE;
              if (this._offset > this._failure) {
                this._failure = this._offset;
                this._expected = [];
              }
              if (this._offset === this._failure) {
                this._expected.push('"="');
              }
            }
            if (address5 !== FAILURE) {
              elements1[1] = address5;
              var address6 = FAILURE;
              address6 = this._read__();
              if (address6 !== FAILURE) {
                elements1[2] = address6;
                var address7 = FAILURE;
                address7 = this._read_Literal();
                if (address7 !== FAILURE) {
                  elements1[3] = address7;
                } else {
                  elements1 = null;
                  this._offset = index3;
                }
              } else {
                elements1 = null;
                this._offset = index3;
              }
            } else {
              elements1 = null;
              this._offset = index3;
            }
          } else {
            elements1 = null;
            this._offset = index3;
          }
          if (elements1 === null) {
            address3 = FAILURE;
          } else {
            address3 = new TreeNode43(this._input.substring(index3, this._offset), index3, elements1);
            this._offset = this._offset;
          }
          if (address3 === FAILURE) {
            address3 = new TreeNode(this._input.substring(index2, index2), index2);
            this._offset = index2;
          }
          if (address3 !== FAILURE) {
            elements0[2] = address3;
            var address8 = FAILURE;
            address8 = this._read__();
            if (address8 !== FAILURE) {
              elements0[3] = address8;
            } else {
              elements0 = null;
              this._offset = index1;
            }
          } else {
            elements0 = null;
            this._offset = index1;
          }
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode42(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._Param[index0] = [address0, this._offset];
      return address0;
    },

    _read_Type: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Type = this._cache._Type || {};
      var cached = this._cache._Type[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      address0 = this._read_id();
      this._cache._Type[index0] = [address0, this._offset];
      return address0;
    },

    _read_Body: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Body = this._cache._Body || {};
      var cached = this._cache._Body[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(4);
      var address1 = FAILURE;
      var chunk0 = null;
      if (this._offset < this._inputSize) {
        chunk0 = this._input.substring(this._offset, this._offset + 1);
      }
      if (chunk0 === '{') {
        address1 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
        this._offset = this._offset + 1;
      } else {
        address1 = FAILURE;
        if (this._offset > this._failure) {
          this._failure = this._offset;
          this._expected = [];
        }
        if (this._offset === this._failure) {
          this._expected.push('"{"');
        }
      }
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        address2 = this._read_Spacing();
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address3 = FAILURE;
          var remaining0 = 0, index2 = this._offset, elements1 = [], address4 = true;
          while (address4 !== FAILURE) {
            address4 = this._read_Stament();
            if (address4 !== FAILURE) {
              elements1.push(address4);
              --remaining0;
            }
          }
          if (remaining0 <= 0) {
            address3 = new TreeNode(this._input.substring(index2, this._offset), index2, elements1);
            this._offset = this._offset;
          } else {
            address3 = FAILURE;
          }
          if (address3 !== FAILURE) {
            elements0[2] = address3;
            var address5 = FAILURE;
            var chunk1 = null;
            if (this._offset < this._inputSize) {
              chunk1 = this._input.substring(this._offset, this._offset + 1);
            }
            if (chunk1 === '}') {
              address5 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
              this._offset = this._offset + 1;
            } else {
              address5 = FAILURE;
              if (this._offset > this._failure) {
                this._failure = this._offset;
                this._expected = [];
              }
              if (this._offset === this._failure) {
                this._expected.push('"}"');
              }
            }
            if (address5 !== FAILURE) {
              elements0[3] = address5;
            } else {
              elements0 = null;
              this._offset = index1;
            }
          } else {
            elements0 = null;
            this._offset = index1;
          }
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode44(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._Body[index0] = [address0, this._offset];
      return address0;
    },

    _read_id: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._id = this._cache._id || {};
      var cached = this._cache._id[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(2);
      var address1 = FAILURE;
      var remaining0 = 1, index2 = this._offset, elements1 = [], address2 = true;
      while (address2 !== FAILURE) {
        var chunk0 = null;
        if (this._offset < this._inputSize) {
          chunk0 = this._input.substring(this._offset, this._offset + 1);
        }
        if (chunk0 !== null && /^[a-zA-Z]/.test(chunk0)) {
          address2 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
          this._offset = this._offset + 1;
        } else {
          address2 = FAILURE;
          if (this._offset > this._failure) {
            this._failure = this._offset;
            this._expected = [];
          }
          if (this._offset === this._failure) {
            this._expected.push('[a-zA-Z]');
          }
        }
        if (address2 !== FAILURE) {
          elements1.push(address2);
          --remaining0;
        }
      }
      if (remaining0 <= 0) {
        address1 = new TreeNode(this._input.substring(index2, this._offset), index2, elements1);
        this._offset = this._offset;
      } else {
        address1 = FAILURE;
      }
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address3 = FAILURE;
        var remaining1 = 0, index3 = this._offset, elements2 = [], address4 = true;
        while (address4 !== FAILURE) {
          var chunk1 = null;
          if (this._offset < this._inputSize) {
            chunk1 = this._input.substring(this._offset, this._offset + 1);
          }
          if (chunk1 !== null && /^[a-zA-Z.]/.test(chunk1)) {
            address4 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
            this._offset = this._offset + 1;
          } else {
            address4 = FAILURE;
            if (this._offset > this._failure) {
              this._failure = this._offset;
              this._expected = [];
            }
            if (this._offset === this._failure) {
              this._expected.push('[a-zA-Z.]');
            }
          }
          if (address4 !== FAILURE) {
            elements2.push(address4);
            --remaining1;
          }
        }
        if (remaining1 <= 0) {
          address3 = new TreeNode(this._input.substring(index3, this._offset), index3, elements2);
          this._offset = this._offset;
        } else {
          address3 = FAILURE;
        }
        if (address3 !== FAILURE) {
          elements0[1] = address3;
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._id[index0] = [address0, this._offset];
      return address0;
    },

    _read_Literal: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Literal = this._cache._Literal || {};
      var cached = this._cache._Literal[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset;
      address0 = this._read_NumLiteral();
      if (address0 === FAILURE) {
        this._offset = index1;
        address0 = this._read_StringLiteral();
        if (address0 === FAILURE) {
          this._offset = index1;
          address0 = this._read_EmbeddedLiteral();
          if (address0 === FAILURE) {
            this._offset = index1;
          }
        }
      }
      this._cache._Literal[index0] = [address0, this._offset];
      return address0;
    },

    _read_EmbeddedLiteral: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._EmbeddedLiteral = this._cache._EmbeddedLiteral || {};
      var cached = this._cache._EmbeddedLiteral[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      address0 = this._read_EmbeddedStatement();
      this._cache._EmbeddedLiteral[index0] = [address0, this._offset];
      return address0;
    },

    _read_NumLiteral: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._NumLiteral = this._cache._NumLiteral || {};
      var cached = this._cache._NumLiteral[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset;
      var index2 = this._offset, elements0 = new Array(4);
      var address1 = FAILURE;
      var index3 = this._offset;
      var chunk0 = null;
      if (this._offset < this._inputSize) {
        chunk0 = this._input.substring(this._offset, this._offset + 1);
      }
      if (chunk0 === '-') {
        address1 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
        this._offset = this._offset + 1;
      } else {
        address1 = FAILURE;
        if (this._offset > this._failure) {
          this._failure = this._offset;
          this._expected = [];
        }
        if (this._offset === this._failure) {
          this._expected.push('"-"');
        }
      }
      if (address1 === FAILURE) {
        address1 = new TreeNode(this._input.substring(index3, index3), index3);
        this._offset = index3;
      }
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        var index4 = this._offset;
        var remaining0 = 0, index5 = this._offset, elements1 = [], address3 = true;
        while (address3 !== FAILURE) {
          var chunk1 = null;
          if (this._offset < this._inputSize) {
            chunk1 = this._input.substring(this._offset, this._offset + 1);
          }
          if (chunk1 !== null && /^[0-9]/.test(chunk1)) {
            address3 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
            this._offset = this._offset + 1;
          } else {
            address3 = FAILURE;
            if (this._offset > this._failure) {
              this._failure = this._offset;
              this._expected = [];
            }
            if (this._offset === this._failure) {
              this._expected.push('[0-9]');
            }
          }
          if (address3 !== FAILURE) {
            elements1.push(address3);
            --remaining0;
          }
        }
        if (remaining0 <= 0) {
          address2 = new TreeNode(this._input.substring(index5, this._offset), index5, elements1);
          this._offset = this._offset;
        } else {
          address2 = FAILURE;
        }
        if (address2 === FAILURE) {
          address2 = new TreeNode(this._input.substring(index4, index4), index4);
          this._offset = index4;
        }
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address4 = FAILURE;
          var index6 = this._offset;
          var chunk2 = null;
          if (this._offset < this._inputSize) {
            chunk2 = this._input.substring(this._offset, this._offset + 1);
          }
          if (chunk2 === '.') {
            address4 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
            this._offset = this._offset + 1;
          } else {
            address4 = FAILURE;
            if (this._offset > this._failure) {
              this._failure = this._offset;
              this._expected = [];
            }
            if (this._offset === this._failure) {
              this._expected.push('"."');
            }
          }
          if (address4 === FAILURE) {
            address4 = new TreeNode(this._input.substring(index6, index6), index6);
            this._offset = index6;
          }
          if (address4 !== FAILURE) {
            elements0[2] = address4;
            var address5 = FAILURE;
            var remaining1 = 1, index7 = this._offset, elements2 = [], address6 = true;
            while (address6 !== FAILURE) {
              var chunk3 = null;
              if (this._offset < this._inputSize) {
                chunk3 = this._input.substring(this._offset, this._offset + 1);
              }
              if (chunk3 !== null && /^[0-9]/.test(chunk3)) {
                address6 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
                this._offset = this._offset + 1;
              } else {
                address6 = FAILURE;
                if (this._offset > this._failure) {
                  this._failure = this._offset;
                  this._expected = [];
                }
                if (this._offset === this._failure) {
                  this._expected.push('[0-9]');
                }
              }
              if (address6 !== FAILURE) {
                elements2.push(address6);
                --remaining1;
              }
            }
            if (remaining1 <= 0) {
              address5 = new TreeNode(this._input.substring(index7, this._offset), index7, elements2);
              this._offset = this._offset;
            } else {
              address5 = FAILURE;
            }
            if (address5 !== FAILURE) {
              elements0[3] = address5;
            } else {
              elements0 = null;
              this._offset = index2;
            }
          } else {
            elements0 = null;
            this._offset = index2;
          }
        } else {
          elements0 = null;
          this._offset = index2;
        }
      } else {
        elements0 = null;
        this._offset = index2;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode(this._input.substring(index2, this._offset), index2, elements0);
        this._offset = this._offset;
      }
      if (address0 === FAILURE) {
        this._offset = index1;
        var remaining2 = 1, index8 = this._offset, elements3 = [], address7 = true;
        while (address7 !== FAILURE) {
          var chunk4 = null;
          if (this._offset < this._inputSize) {
            chunk4 = this._input.substring(this._offset, this._offset + 1);
          }
          if (chunk4 !== null && /^[0-9]/.test(chunk4)) {
            address7 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
            this._offset = this._offset + 1;
          } else {
            address7 = FAILURE;
            if (this._offset > this._failure) {
              this._failure = this._offset;
              this._expected = [];
            }
            if (this._offset === this._failure) {
              this._expected.push('[0-9]');
            }
          }
          if (address7 !== FAILURE) {
            elements3.push(address7);
            --remaining2;
          }
        }
        if (remaining2 <= 0) {
          address0 = new TreeNode(this._input.substring(index8, this._offset), index8, elements3);
          this._offset = this._offset;
        } else {
          address0 = FAILURE;
        }
        if (address0 === FAILURE) {
          this._offset = index1;
        }
      }
      this._cache._NumLiteral[index0] = [address0, this._offset];
      return address0;
    },

    _read_StringLiteral: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._StringLiteral = this._cache._StringLiteral || {};
      var cached = this._cache._StringLiteral[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset;
      var index2 = this._offset, elements0 = new Array(3);
      var address1 = FAILURE;
      var chunk0 = null;
      if (this._offset < this._inputSize) {
        chunk0 = this._input.substring(this._offset, this._offset + 1);
      }
      if (chunk0 === '\'') {
        address1 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
        this._offset = this._offset + 1;
      } else {
        address1 = FAILURE;
        if (this._offset > this._failure) {
          this._failure = this._offset;
          this._expected = [];
        }
        if (this._offset === this._failure) {
          this._expected.push('"\'"');
        }
      }
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        var remaining0 = 1, index3 = this._offset, elements1 = [], address3 = true;
        while (address3 !== FAILURE) {
          var chunk1 = null;
          if (this._offset < this._inputSize) {
            chunk1 = this._input.substring(this._offset, this._offset + 1);
          }
          if (chunk1 !== null && /^[^\'\"]/.test(chunk1)) {
            address3 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
            this._offset = this._offset + 1;
          } else {
            address3 = FAILURE;
            if (this._offset > this._failure) {
              this._failure = this._offset;
              this._expected = [];
            }
            if (this._offset === this._failure) {
              this._expected.push('[^\\\'\\"]');
            }
          }
          if (address3 !== FAILURE) {
            elements1.push(address3);
            --remaining0;
          }
        }
        if (remaining0 <= 0) {
          address2 = new TreeNode(this._input.substring(index3, this._offset), index3, elements1);
          this._offset = this._offset;
        } else {
          address2 = FAILURE;
        }
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address4 = FAILURE;
          var chunk2 = null;
          if (this._offset < this._inputSize) {
            chunk2 = this._input.substring(this._offset, this._offset + 1);
          }
          if (chunk2 === '\'') {
            address4 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
            this._offset = this._offset + 1;
          } else {
            address4 = FAILURE;
            if (this._offset > this._failure) {
              this._failure = this._offset;
              this._expected = [];
            }
            if (this._offset === this._failure) {
              this._expected.push('"\'"');
            }
          }
          if (address4 !== FAILURE) {
            elements0[2] = address4;
          } else {
            elements0 = null;
            this._offset = index2;
          }
        } else {
          elements0 = null;
          this._offset = index2;
        }
      } else {
        elements0 = null;
        this._offset = index2;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode(this._input.substring(index2, this._offset), index2, elements0);
        this._offset = this._offset;
      }
      if (address0 === FAILURE) {
        this._offset = index1;
        var index4 = this._offset, elements2 = new Array(3);
        var address5 = FAILURE;
        var chunk3 = null;
        if (this._offset < this._inputSize) {
          chunk3 = this._input.substring(this._offset, this._offset + 1);
        }
        if (chunk3 === '"') {
          address5 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
          this._offset = this._offset + 1;
        } else {
          address5 = FAILURE;
          if (this._offset > this._failure) {
            this._failure = this._offset;
            this._expected = [];
          }
          if (this._offset === this._failure) {
            this._expected.push('"\\""');
          }
        }
        if (address5 !== FAILURE) {
          elements2[0] = address5;
          var address6 = FAILURE;
          var remaining1 = 1, index5 = this._offset, elements3 = [], address7 = true;
          while (address7 !== FAILURE) {
            var chunk4 = null;
            if (this._offset < this._inputSize) {
              chunk4 = this._input.substring(this._offset, this._offset + 1);
            }
            if (chunk4 !== null && /^[^\"]/.test(chunk4)) {
              address7 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
              this._offset = this._offset + 1;
            } else {
              address7 = FAILURE;
              if (this._offset > this._failure) {
                this._failure = this._offset;
                this._expected = [];
              }
              if (this._offset === this._failure) {
                this._expected.push('[^\\"]');
              }
            }
            if (address7 !== FAILURE) {
              elements3.push(address7);
              --remaining1;
            }
          }
          if (remaining1 <= 0) {
            address6 = new TreeNode(this._input.substring(index5, this._offset), index5, elements3);
            this._offset = this._offset;
          } else {
            address6 = FAILURE;
          }
          if (address6 !== FAILURE) {
            elements2[1] = address6;
            var address8 = FAILURE;
            var chunk5 = null;
            if (this._offset < this._inputSize) {
              chunk5 = this._input.substring(this._offset, this._offset + 1);
            }
            if (chunk5 === '"') {
              address8 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
              this._offset = this._offset + 1;
            } else {
              address8 = FAILURE;
              if (this._offset > this._failure) {
                this._failure = this._offset;
                this._expected = [];
              }
              if (this._offset === this._failure) {
                this._expected.push('"\\""');
              }
            }
            if (address8 !== FAILURE) {
              elements2[2] = address8;
            } else {
              elements2 = null;
              this._offset = index4;
            }
          } else {
            elements2 = null;
            this._offset = index4;
          }
        } else {
          elements2 = null;
          this._offset = index4;
        }
        if (elements2 === null) {
          address0 = FAILURE;
        } else {
          address0 = new TreeNode(this._input.substring(index4, this._offset), index4, elements2);
          this._offset = this._offset;
        }
        if (address0 === FAILURE) {
          this._offset = index1;
        }
      }
      this._cache._StringLiteral[index0] = [address0, this._offset];
      return address0;
    },

    _read_Spacing: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Spacing = this._cache._Spacing || {};
      var cached = this._cache._Spacing[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var remaining0 = 0, index1 = this._offset, elements0 = [], address1 = true;
      while (address1 !== FAILURE) {
        var index2 = this._offset;
        var chunk0 = null;
        if (this._offset < this._inputSize) {
          chunk0 = this._input.substring(this._offset, this._offset + 1);
        }
        if (chunk0 !== null && /^[\s]/.test(chunk0)) {
          address1 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
          this._offset = this._offset + 1;
        } else {
          address1 = FAILURE;
          if (this._offset > this._failure) {
            this._failure = this._offset;
            this._expected = [];
          }
          if (this._offset === this._failure) {
            this._expected.push('[\\s]');
          }
        }
        if (address1 === FAILURE) {
          this._offset = index2;
          address1 = this._read_Comment();
          if (address1 === FAILURE) {
            this._offset = index2;
          }
        }
        if (address1 !== FAILURE) {
          elements0.push(address1);
          --remaining0;
        }
      }
      if (remaining0 <= 0) {
        address0 = new TreeNode(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      } else {
        address0 = FAILURE;
      }
      this._cache._Spacing[index0] = [address0, this._offset];
      return address0;
    },

    _read_Comment: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._Comment = this._cache._Comment || {};
      var cached = this._cache._Comment[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset, elements0 = new Array(3);
      var address1 = FAILURE;
      var chunk0 = null;
      if (this._offset < this._inputSize) {
        chunk0 = this._input.substring(this._offset, this._offset + 2);
      }
      if (chunk0 === '//') {
        address1 = new TreeNode(this._input.substring(this._offset, this._offset + 2), this._offset);
        this._offset = this._offset + 2;
      } else {
        address1 = FAILURE;
        if (this._offset > this._failure) {
          this._failure = this._offset;
          this._expected = [];
        }
        if (this._offset === this._failure) {
          this._expected.push('\'//\'');
        }
      }
      if (address1 !== FAILURE) {
        elements0[0] = address1;
        var address2 = FAILURE;
        var remaining0 = 0, index2 = this._offset, elements1 = [], address3 = true;
        while (address3 !== FAILURE) {
          var index3 = this._offset, elements2 = new Array(2);
          var address4 = FAILURE;
          var index4 = this._offset;
          address4 = this._read_EndOfLine();
          this._offset = index4;
          if (address4 === FAILURE) {
            address4 = new TreeNode(this._input.substring(this._offset, this._offset), this._offset);
            this._offset = this._offset;
          } else {
            address4 = FAILURE;
          }
          if (address4 !== FAILURE) {
            elements2[0] = address4;
            var address5 = FAILURE;
            if (this._offset < this._inputSize) {
              address5 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
              this._offset = this._offset + 1;
            } else {
              address5 = FAILURE;
              if (this._offset > this._failure) {
                this._failure = this._offset;
                this._expected = [];
              }
              if (this._offset === this._failure) {
                this._expected.push('<any char>');
              }
            }
            if (address5 !== FAILURE) {
              elements2[1] = address5;
            } else {
              elements2 = null;
              this._offset = index3;
            }
          } else {
            elements2 = null;
            this._offset = index3;
          }
          if (elements2 === null) {
            address3 = FAILURE;
          } else {
            address3 = new TreeNode(this._input.substring(index3, this._offset), index3, elements2);
            this._offset = this._offset;
          }
          if (address3 !== FAILURE) {
            elements1.push(address3);
            --remaining0;
          }
        }
        if (remaining0 <= 0) {
          address2 = new TreeNode(this._input.substring(index2, this._offset), index2, elements1);
          this._offset = this._offset;
        } else {
          address2 = FAILURE;
        }
        if (address2 !== FAILURE) {
          elements0[1] = address2;
          var address6 = FAILURE;
          address6 = this._read_EndOfLine();
          if (address6 !== FAILURE) {
            elements0[2] = address6;
          } else {
            elements0 = null;
            this._offset = index1;
          }
        } else {
          elements0 = null;
          this._offset = index1;
        }
      } else {
        elements0 = null;
        this._offset = index1;
      }
      if (elements0 === null) {
        address0 = FAILURE;
      } else {
        address0 = new TreeNode45(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      }
      this._cache._Comment[index0] = [address0, this._offset];
      return address0;
    },

    _read__: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache.__ = this._cache.__ || {};
      var cached = this._cache.__[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var remaining0 = 0, index1 = this._offset, elements0 = [], address1 = true;
      while (address1 !== FAILURE) {
        var chunk0 = null;
        if (this._offset < this._inputSize) {
          chunk0 = this._input.substring(this._offset, this._offset + 1);
        }
        if (chunk0 !== null && /^[\s]/.test(chunk0)) {
          address1 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
          this._offset = this._offset + 1;
        } else {
          address1 = FAILURE;
          if (this._offset > this._failure) {
            this._failure = this._offset;
            this._expected = [];
          }
          if (this._offset === this._failure) {
            this._expected.push('[\\s]');
          }
        }
        if (address1 !== FAILURE) {
          elements0.push(address1);
          --remaining0;
        }
      }
      if (remaining0 <= 0) {
        address0 = new TreeNode(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      } else {
        address0 = FAILURE;
      }
      this._cache.__[index0] = [address0, this._offset];
      return address0;
    },

    _read___: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache.___ = this._cache.___ || {};
      var cached = this._cache.___[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var remaining0 = 1, index1 = this._offset, elements0 = [], address1 = true;
      while (address1 !== FAILURE) {
        var chunk0 = null;
        if (this._offset < this._inputSize) {
          chunk0 = this._input.substring(this._offset, this._offset + 1);
        }
        if (chunk0 !== null && /^[\s]/.test(chunk0)) {
          address1 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
          this._offset = this._offset + 1;
        } else {
          address1 = FAILURE;
          if (this._offset > this._failure) {
            this._failure = this._offset;
            this._expected = [];
          }
          if (this._offset === this._failure) {
            this._expected.push('[\\s]');
          }
        }
        if (address1 !== FAILURE) {
          elements0.push(address1);
          --remaining0;
        }
      }
      if (remaining0 <= 0) {
        address0 = new TreeNode(this._input.substring(index1, this._offset), index1, elements0);
        this._offset = this._offset;
      } else {
        address0 = FAILURE;
      }
      this._cache.___[index0] = [address0, this._offset];
      return address0;
    },

    _read_EndOfLine: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._EndOfLine = this._cache._EndOfLine || {};
      var cached = this._cache._EndOfLine[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset;
      var chunk0 = null;
      if (this._offset < this._inputSize) {
        chunk0 = this._input.substring(this._offset, this._offset + 2);
      }
      if (chunk0 === '\r\n') {
        address0 = new TreeNode(this._input.substring(this._offset, this._offset + 2), this._offset);
        this._offset = this._offset + 2;
      } else {
        address0 = FAILURE;
        if (this._offset > this._failure) {
          this._failure = this._offset;
          this._expected = [];
        }
        if (this._offset === this._failure) {
          this._expected.push('\'\\r\\n\'');
        }
      }
      if (address0 === FAILURE) {
        this._offset = index1;
        var chunk1 = null;
        if (this._offset < this._inputSize) {
          chunk1 = this._input.substring(this._offset, this._offset + 1);
        }
        if (chunk1 === '\n') {
          address0 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
          this._offset = this._offset + 1;
        } else {
          address0 = FAILURE;
          if (this._offset > this._failure) {
            this._failure = this._offset;
            this._expected = [];
          }
          if (this._offset === this._failure) {
            this._expected.push('\'\\n\'');
          }
        }
        if (address0 === FAILURE) {
          this._offset = index1;
          var chunk2 = null;
          if (this._offset < this._inputSize) {
            chunk2 = this._input.substring(this._offset, this._offset + 1);
          }
          if (chunk2 === '\r') {
            address0 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
            this._offset = this._offset + 1;
          } else {
            address0 = FAILURE;
            if (this._offset > this._failure) {
              this._failure = this._offset;
              this._expected = [];
            }
            if (this._offset === this._failure) {
              this._expected.push('\'\\r\'');
            }
          }
          if (address0 === FAILURE) {
            this._offset = index1;
          }
        }
      }
      this._cache._EndOfLine[index0] = [address0, this._offset];
      return address0;
    },

    _read_EndOfFile: function() {
      var address0 = FAILURE, index0 = this._offset;
      this._cache._EndOfFile = this._cache._EndOfFile || {};
      var cached = this._cache._EndOfFile[index0];
      if (cached) {
        this._offset = cached[1];
        return cached[0];
      }
      var index1 = this._offset;
      if (this._offset < this._inputSize) {
        address0 = new TreeNode(this._input.substring(this._offset, this._offset + 1), this._offset);
        this._offset = this._offset + 1;
      } else {
        address0 = FAILURE;
        if (this._offset > this._failure) {
          this._failure = this._offset;
          this._expected = [];
        }
        if (this._offset === this._failure) {
          this._expected.push('<any char>');
        }
      }
      this._offset = index1;
      if (address0 === FAILURE) {
        address0 = new TreeNode(this._input.substring(this._offset, this._offset), this._offset);
        this._offset = this._offset;
      } else {
        address0 = FAILURE;
      }
      this._cache._EndOfFile[index0] = [address0, this._offset];
      return address0;
    }
  };

  var Parser = function(input, actions, types) {
    this._input = input;
    this._inputSize = input.length;
    this._actions = actions;
    this._types = types;
    this._offset = 0;
    this._cache = {};
    this._failure = 0;
    this._expected = [];
  };

  Parser.prototype.parse = function() {
    var tree = this._read_Grammar();
    if (tree !== FAILURE && this._offset === this._inputSize) {
      return tree;
    }
    if (this._expected.length === 0) {
      this._failure = this._offset;
      this._expected.push('<EOF>');
    }
    this.constructor.lastError = {offset: this._offset, expected: this._expected};
      return { 'input' : this._input, 'failure' : this._failure, 'expected' : this._expected};
  };

  var parse = function(input, options) {
    options = options || {};
    var parser = new Parser(input, options.actions, options.types);
    return parser.parse();
  };
  extend(Parser.prototype, Grammar);

  var exported = {Grammar: Grammar, Parser: Parser, parse: parse};

  if (typeof require === 'function' && typeof exports === 'object') {
    extend(exports, exported);
  } else {
    var namespace = typeof this !== 'undefined' ? this : window;
    namespace.LinguaFranca = exported;
  }
})();
