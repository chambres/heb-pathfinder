# HEBpath

**HEBpath** computes the fastest route through H-E-B to collect every item in your cart. All computation happens in the browser using a Rust → WASM engine.

## How It Works

### The Shortcut (iOS)

The Shortcut pulls item location data directly from your H-E-B cart page:

1. Runs JavaScript on the cart page to extract each item's aisle, section, and location metadata  
2. Compresses everything with **lz-string**  
3. Passes the compressed data to the HEBpath site via the URL fragment  

On desktop, you can skip the Shortcut and paste the JS snippet from the site into the console. (Enable console pasting in Chrome.)

### In the Browser (Rust + WASM)

Once the data arrives, the WebAssembly module processes it:

1. Maps each item onto the store’s SVG  
2. Rasterizes the SVG into a grid  
3. Runs an A* search to compute the shortest path  
4. Returns the computed route  
5. Renders the path and labels onto the SVG  

Everything runs locally — no backend requests and no server processing.

### Caching

The site caches your last computed path so reloads don’t require recalculating.

## Try It

**https://rhl.sh/heb**
