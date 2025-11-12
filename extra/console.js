var LZString=function(){var r=String.fromCharCode,o="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/=",n="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+-$",e={};function t(r,o){if(!e[r]){e[r]={};for(var n=0;n<r.length;n++)e[r][r.charAt(n)]=n}return e[r][o]}var i={compressToBase64:function(r){if(null==r)return"";var n=i._compress(r,6,function(r){return o.charAt(r)});switch(n.length%4){default:case 0:return n;case 1:return n+"===";case 2:return n+"==";case 3:return n+"="}},decompressFromBase64:function(r){return null==r?"":""==r?null:i._decompress(r.length,32,function(n){return t(o,r.charAt(n))})},compressToUTF16:function(o){return null==o?"":i._compress(o,15,function(o){return r(o+32)})+" "},decompressFromUTF16:function(r){return null==r?"":""==r?null:i._decompress(r.length,16384,function(o){return r.charCodeAt(o)-32})},compressToUint8Array:function(r){for(var o=i.compress(r),n=new Uint8Array(2*o.length),e=0,t=o.length;e<t;e++){var s=o.charCodeAt(e);n[2*e]=s>>>8,n[2*e+1]=s%256}return n},decompressFromUint8Array:function(o){if(null==o)return i.decompress(o);for(var n=new Array(o.length/2),e=0,t=n.length;e<t;e++)n[e]=256*o[2*e]+o[2*e+1];var s=[];return n.forEach(function(o){s.push(r(o))}),i.decompress(s.join(""))},compressToEncodedURIComponent:function(r){return null==r?"":i._compress(r,6,function(r){return n.charAt(r)})},decompressFromEncodedURIComponent:function(r){return null==r?"":""==r?null:(r=r.replace(/ /g,"+"),i._decompress(r.length,32,function(o){return t(n,r.charAt(o))}))},compress:function(o){return i._compress(o,16,function(o){return r(o)})},_compress:function(r,o,n){if(null==r)return"";var e,t,i,s={},u={},a="",p="",c="",l=2,f=3,h=2,d=[],m=0,v=0;for(i=0;i<r.length;i+=1)if(a=r.charAt(i),Object.prototype.hasOwnProperty.call(s,a)||(s[a]=f++,u[a]=!0),p=c+a,Object.prototype.hasOwnProperty.call(s,p))c=p;else{if(Object.prototype.hasOwnProperty.call(u,c)){if(c.charCodeAt(0)<256){for(e=0;e<h;e++)m<<=1,v==o-1?(v=0,d.push(n(m)),m=0):v++;for(t=c.charCodeAt(0),e=0;e<8;e++)m=m<<1|1&t,v==o-1?(v=0,d.push(n(m)),m=0):v++,t>>=1}else{for(t=1,e=0;e<h;e++)m=m<<1|t,v==o-1?(v=0,d.push(n(m)),m=0):v++,t=0;for(t=c.charCodeAt(0),e=0;e<16;e++)m=m<<1|1&t,v==o-1?(v=0,d.push(n(m)),m=0):v++,t>>=1}0==--l&&(l=Math.pow(2,h),h++),delete u[c]}else for(t=s[c],e=0;e<h;e++)m=m<<1|1&t,v==o-1?(v=0,d.push(n(m)),m=0):v++,t>>=1;0==--l&&(l=Math.pow(2,h),h++),s[p]=f++,c=String(a)}if(""!==c){if(Object.prototype.hasOwnProperty.call(u,c)){if(c.charCodeAt(0)<256){for(e=0;e<h;e++)m<<=1,v==o-1?(v=0,d.push(n(m)),m=0):v++;for(t=c.charCodeAt(0),e=0;e<8;e++)m=m<<1|1&t,v==o-1?(v=0,d.push(n(m)),m=0):v++,t>>=1}else{for(t=1,e=0;e<h;e++)m=m<<1|t,v==o-1?(v=0,d.push(n(m)),m=0):v++,t=0;for(t=c.charCodeAt(0),e=0;e<16;e++)m=m<<1|1&t,v==o-1?(v=0,d.push(n(m)),m=0):v++,t>>=1}0==--l&&(l=Math.pow(2,h),h++),delete u[c]}else for(t=s[c],e=0;e<h;e++)m=m<<1|1&t,v==o-1?(v=0,d.push(n(m)),m=0):v++,t>>=1;0==--l&&(l=Math.pow(2,h),h++)}for(t=2,e=0;e<h;e++)m=m<<1|1&t,v==o-1?(v=0,d.push(n(m)),m=0):v++,t>>=1;for(;;){if(m<<=1,v==o-1){d.push(n(m));break}v++}return d.join("")},decompress:function(r){return null==r?"":""==r?null:i._decompress(r.length,32768,function(o){return r.charCodeAt(o)})},_decompress:function(o,n,e){var t,i,s,u,a,p,c,l=[],f=4,h=4,d=3,m="",v=[],g={val:e(0),position:n,index:1};for(t=0;t<3;t+=1)l[t]=t;for(s=0,a=Math.pow(2,2),p=1;p!=a;)u=g.val&g.position,g.position>>=1,0==g.position&&(g.position=n,g.val=e(g.index++)),s|=(u>0?1:0)*p,p<<=1;switch(s){case 0:for(s=0,a=Math.pow(2,8),p=1;p!=a;)u=g.val&g.position,g.position>>=1,0==g.position&&(g.position=n,g.val=e(g.index++)),s|=(u>0?1:0)*p,p<<=1;c=r(s);break;case 1:for(s=0,a=Math.pow(2,16),p=1;p!=a;)u=g.val&g.position,g.position>>=1,0==g.position&&(g.position=n,g.val=e(g.index++)),s|=(u>0?1:0)*p,p<<=1;c=r(s);break;case 2:return""}for(l[3]=c,i=c,v.push(c);;){if(g.index>o)return"";for(s=0,a=Math.pow(2,d),p=1;p!=a;)u=g.val&g.position,g.position>>=1,0==g.position&&(g.position=n,g.val=e(g.index++)),s|=(u>0?1:0)*p,p<<=1;switch(c=s){case 0:for(s=0,a=Math.pow(2,8),p=1;p!=a;)u=g.val&g.position,g.position>>=1,0==g.position&&(g.position=n,g.val=e(g.index++)),s|=(u>0?1:0)*p,p<<=1;l[h++]=r(s),c=h-1,f--;break;case 1:for(s=0,a=Math.pow(2,16),p=1;p!=a;)u=g.val&g.position,g.position>>=1,0==g.position&&(g.position=n,g.val=e(g.index++)),s|=(u>0?1:0)*p,p<<=1;l[h++]=r(s),c=h-1,f--;break;case 2:return v.join("")}if(0==f&&(f=Math.pow(2,d),d++),l[c])m=l[c];else{if(c!==h)return null;m=i+i.charAt(0)}v.push(m),l[h++]=i+m.charAt(0),i=m,0==--f&&(f=Math.pow(2,d),d++)}}};return i}();"function"==typeof define&&define.amd?define(function(){return LZString}):"undefined"!=typeof module&&null!=module?module.exports=LZString:"undefined"!=typeof angular&&null!=angular&&angular.module("LZString",[]).factory("LZString",function(){return LZString});
 function waitForElements(selector, {
  root = document,
  min = 1,
  timeoutMs = 30000
} = {}) {
  return new Promise((resolve, reject) => {
    const check = () => {
      const nodes = Array.from(root.querySelectorAll(selector));
      if (nodes.length >= min) {
        observer.disconnect();
        resolve(nodes);
      }
    };

    const observer = new MutationObserver(check);
    observer.observe(root, { childList: true, subtree: true, attributes: true });

    // initial pass (in case they're already there)
    check();

    const t = setTimeout(() => {
      observer.disconnect();
      reject(new Error(`Timed out waiting for selector: "${selector}"`));
    }, timeoutMs);

    // Clean up timer on resolve
    const origResolve = resolve;
    resolve = (v) => { clearTimeout(t); origResolve(v); };
  });
}

function waitForElement(selector, opts = {}) {
  return waitForElements(selector, { ...opts, min: 1 }).then(nodes => nodes[0]);
}

function waitForPredicate(predicate, { root = document, timeoutMs = 30000 } = {}) {
  return new Promise((resolve, reject) => {
    const check = () => {
      try {
        const node = predicate();
        if (node) {
          observer.disconnect();
          clearTimeout(t);
          resolve(node);
        }
      } catch (e) {
        // ignore predicate errors while DOM settles
      }
    };

    const observer = new MutationObserver(check);
    observer.observe(root, { childList: true, subtree: true, attributes: true });

    // initial pass
    check();

    const t = setTimeout(() => {
      observer.disconnect();
      reject(new Error('Timed out waiting for predicate to return a truthy value.'));
    }, timeoutMs);
  });
}

(async () => {
  try {
    let storeId = '543'; // default store ID

    // Grab product anchors
    const elements = await waitForElements('[data-qe-id="itemRowDetailsName"]', { min: 1, timeoutMs: 30000 });

    // Build an ID -> title map and a list of product IDs (last path segment fallback)
    const idTitleMap = new Map();
    const productIds = elements.map(el => {
      const href = el.href || el.getAttribute('href') || '';
      const parts = href.split('/').filter(Boolean);
      const lastPart = parts[parts.length - 1] || null;

      // Prefer numeric ID from element id attribute if present, else fallback to lastPart
      const id = (el.id && el.id.trim()) || lastPart;

      // Title attribute on the anchor contains the product title we want
      const title = el.getAttribute('title') || el.textContent?.trim() || null;

      if (id) idTitleMap.set(id, title);
      return id;
    }).filter(Boolean);

    console.log('productIds:', productIds);

    // Open reservation modal & change store flow (unchanged)
    const chooseBtn = await waitForElement('button[data-qe-id="chooseReservationTime"]', { timeoutMs: 30000 });
    chooseBtn.click();

    const changeStoreBtn = await waitForElement('button[data-testid="fulfillment_change_store"]', { timeoutMs: 30000 });
    changeStoreBtn.click();

    const span = await waitForPredicate(() => {
      return Array.from(document.querySelectorAll('span'))
        .find(el => el.textContent && el.textContent.includes(' Selected Store'));
    }, { timeoutMs: 30000 });

    const li = span.closest('li');
    const link = li ? li.querySelector('a[href]') : null;
    const href = link ? link.href : null;

    if (href) {
      // split by '-' and get last part
      storeId = href.split('-').filter(Boolean).pop();
      console.log('Extracted store ID:', storeId);
    } else {
      console.log('No link found inside the selected store <li>.');
    }

    const closebtn = await waitForElement('button[data-qe-id="modalClose"]', { timeoutMs: 30000 });
    closebtn.click();

    // Fetch product JSON for each ID and add title field (keeping original shape + title)
    const allData = await Promise.all(
      productIds.map(async (id) => {
        try {
          const res = await fetch(`https://www.heb.com/pals/v2.0/location/store/${storeId}/products/${id}`, {
            credentials: "include"
          });
          const obj = await res.json();
          // Preserve original object, just add title
          return { ...obj, title: idTitleMap.get(id) ?? null };
        } catch (err) {
          console.error("Error fetching ID:", id, err);
          return { id, title: idTitleMap.get(id) ?? null, error: String(err) };
        }
      })
    );

    // Log combined JSON (data + svg), formatted
    // --- SVG fetch & cleaning (unchanged) ---
    const url = "https://www.heb.com/atlas/v1.0/image?locationNumber=543&format=svg&style=none&label=false&drawPsas=false&drawAisleMarkers=false&drawCombinedFixtures=true&drawDepartmentLabels=false&hidePartnerFixtures=true";
    const resp = await fetch(url, { credentials: "include" });
    if (!resp.ok) throw new Error(`Fetch failed: ${resp.status} ${resp.statusText}`);
    const svgText = await resp.text();

    const parser = new DOMParser();
    const doc = parser.parseFromString(svgText, "image/svg+xml");
    if (doc.querySelector("parsererror")) {
      console.error(doc.querySelector("parsererror")?.textContent || "Parser error");
      throw new Error("Invalid SVG document");
    }

    let removedCount = 0;
    doc.querySelectorAll("polygon").forEach(poly => {
      poly.querySelectorAll("title").forEach(t => {
        const text = (t.textContent || "").trim().toUpperCase();
        if (!text.includes("CHECKSTAND MERCHANDISER SELF CHECK OUT")) {
          t.remove();
          removedCount++;
        }
      });
    });

    const out = new XMLSerializer().serializeToString(doc);
    console.log(`Done. Removed ${removedCount} <title> element(s) from polygons.`);

    const combined = { data: allData, svg: out };
    const jsonText = JSON.stringify(combined, null, 2);
    console.log("Combined JSON:", jsonText);

    window.open("http://localhost:8000/index.html#{" + LZString.compressToEncodedURIComponent(jsonText) + "}");

  } catch (e) {
    console.error(e);
  }
})();
