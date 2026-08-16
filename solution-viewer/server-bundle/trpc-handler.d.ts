// Types for trpc-handler.js, which `npm run build:server` generates into this
// directory and .gitignore keeps out of the repo. Declaring the shape here
// rather than generating it keeps a fresh clone typecheckable before any build
// has run; forwarding to the bundle's own entry point keeps the two in step.
export { default } from "../src/server/vercel-handler";
