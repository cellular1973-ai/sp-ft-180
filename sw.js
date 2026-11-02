// SP-FT-180 - cevrimdisi onbellek. Uygulama tek dosya; surum degisince CACHE adini artirin.
const CACHE = "sp-ft-180-v1";
const FILES = [
  "./sp_ft_180.html",
  "./index.html",
  "./spft180.html",
  "./spft180.webmanifest",
  "./icons/spft180/icon-192.png",
  "./icons/spft180/icon-512.png",
  "./icons/spft180/apple-touch-icon.png",
];

self.addEventListener("install", (e) => {
  e.waitUntil(caches.open(CACHE).then((c) => c.addAll(FILES)).then(() => self.skipWaiting()));
});

self.addEventListener("activate", (e) => {
  e.waitUntil(
    caches.keys()
      .then((keys) => Promise.all(keys.filter((k) => k !== CACHE).map((k) => caches.delete(k))))
      .then(() => self.clients.claim())
  );
});

// once ag (guncel surum), ag yoksa onbellek
self.addEventListener("fetch", (e) => {
  if (e.request.method !== "GET") return;
  e.respondWith(
    fetch(e.request)
      .then((res) => {
        if (res.ok && new URL(e.request.url).origin === location.origin) {
          const copy = res.clone();
          caches.open(CACHE).then((c) => c.put(e.request, copy));
        }
        return res;
      })
      .catch(() => caches.match(e.request, { ignoreSearch: true }))
  );
});
