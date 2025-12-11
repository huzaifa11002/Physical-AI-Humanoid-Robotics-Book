module.exports = {
    testEnvironment: 'jsdom',
    testMatch: ['**/__tests__/**/*.[jt]s?(x)', '**/?(*.)+(spec|test).[jt]s?(x)'],
    moduleNameMapper: {
        '\\.(css|less|scss|sass)$': 'identity-obj-proxy',
        '@docusaurus/(.*)': 'identity-obj-proxy',
        '@site/(.*)': '<rootDir>/$1',
        '@theme/(.*)': 'identity-obj-proxy',
    },
    transform: {
        '^.+\\.[jt]sx?$': 'babel-jest',
    },
};
