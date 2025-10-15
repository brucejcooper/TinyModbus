import {indexPage} from "./app/routes/index.js"
import {render} from '@lit-labs/ssr';
import {RenderResultReadable} from '@lit-labs/ssr/lib/render-result-readable.js';

export default {
//   plugins: [esbuildPlugin({ ts: true })],
  watch: true,
  rootDir: "app",
  nodeResolve: {
    exportConditions: ["development"],
  },
  middleware: [
    async (ctx, next) => {
        let tmpl = undefined;
        if (ctx.request.path == "/") {
            tmpl = indexPage();
        }

        if (tmpl) {
            ctx.type = "text/html";
            ctx.body = new RenderResultReadable(render(tmpl));
        } else {
            await next();
        }
    }
  ]
};
