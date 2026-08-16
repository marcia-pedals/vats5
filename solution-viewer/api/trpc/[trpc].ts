// Vercel does not bundle functions the way a framework does: it compiles each
// .ts file it can reach to .js in place and leaves the import specifiers
// untouched, so extensionless relative imports — fine under tsx and Vite —
// cannot be resolved by Node in the deployed ES module.
//
// Rather than obey those rules everywhere, this function imports exactly one
// thing: the pre-bundled server that `npm run build` writes to server-bundle/.
// Vercel runs the build command before it compiles anything under api/, so the
// file is on disk by then. Any new function here should follow the same shape —
// re-export from a bundle, never import src/ directly.
export { default } from "../../server-bundle/trpc-handler.js";
