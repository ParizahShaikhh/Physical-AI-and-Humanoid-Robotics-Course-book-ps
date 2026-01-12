const path = require('path');

module.exports = function (context, options) {
  return {
    name: 'docusaurus-plugin-global-components',

    getClientModules() {
      return [path.resolve(__dirname, './GlobalComponents.jsx')];
    },
  };
};