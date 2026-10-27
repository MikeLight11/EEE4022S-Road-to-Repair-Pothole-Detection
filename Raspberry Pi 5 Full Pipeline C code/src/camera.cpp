/*
 * camera.cpp - libcamera capture behind the C interface in camera.h.
 *
 * libcamera only has a C++ API, so this is the one C++ file in the project.
 * It is kept to setup, buffer mapping and a request queue; all processing
 * happens in C on the frames it hands out.
 */
#include "camera.h"

#include <libcamera/libcamera.h>
#include <sys/mman.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

using namespace libcamera;

struct MappedBuffer {
    uint8_t *plane[3];
};

struct cam {
    std::unique_ptr<CameraManager> cm;
    std::shared_ptr<Camera> camera;
    std::unique_ptr<CameraConfiguration> config;
    std::unique_ptr<FrameBufferAllocator> alloc;
    Stream *main_stream = nullptr;
    Stream *lores_stream = nullptr;
    std::vector<std::unique_ptr<Request>> requests;
    std::vector<cam_frame> frames;                    /* one per request, indexed by cookie */
    std::map<const FrameBuffer *, MappedBuffer> mapped;
    std::vector<std::pair<void *, size_t>> mmaps;
    int64_t frame_us = 33333;
    bool acquired = false, started = false;

    std::mutex m;
    std::condition_variable cv;
    std::deque<Request *> done;
    unsigned skipped = 0;

    /* Called on libcamera's thread when the ISP finishes a frame. */
    void on_complete(Request *r)
    {
        if (r->status() == Request::RequestCancelled)
            return;
        {
            std::lock_guard<std::mutex> l(m);
            done.push_back(r);
        }
        cv.notify_one();
    }

    void requeue(Request *r)
    {
        r->reuse(Request::ReuseBuffers);
        camera->queueRequest(r);
    }
};

/* mmap every plane of a dmabuf-backed buffer. Planes usually share one fd
 * at different offsets, so each fd is mapped once over its full extent. */
static int map_buffer(cam_t *c, const FrameBuffer *b)
{
    const auto &pl = b->planes();
    MappedBuffer mb{};
    void *base[3] = {};

    for (size_t i = 0; i < pl.size() && i < 3; i++) {
        int fd = pl[i].fd.get();
        for (size_t j = 0; j < i; j++)
            if (pl[j].fd.get() == fd)
                base[i] = base[j];
        if (!base[i]) {
            size_t len = 0;
            for (const auto &q : pl)
                if (q.fd.get() == fd)
                    len = std::max(len, (size_t)q.offset + q.length);
            void *p = mmap(nullptr, len, PROT_READ, MAP_SHARED, fd, 0);
            if (p == MAP_FAILED) {
                perror("camera: mmap");
                return -1;
            }
            c->mmaps.emplace_back(p, len);
            base[i] = p;
        }
        mb.plane[i] = (uint8_t *)base[i] + pl[i].offset;
    }
    c->mapped[b] = mb;
    return 0;
}

static void fill_image(cam_t *c, Request *r, Stream *s, cam_image *img)
{
    const StreamConfiguration &sc = s->configuration();
    const FrameBuffer *b = r->findBuffer(s);
    const MappedBuffer &mb = c->mapped[b];

    img->width = sc.size.width;
    img->height = sc.size.height;
    img->stride = sc.stride;
    img->uv_stride = sc.stride / 2;
    img->y = mb.plane[0];
    if (b->planes().size() >= 3) {
        img->u = mb.plane[1];
        img->v = mb.plane[2];
    } else { /* single-plane buffer: U and V follow Y contiguously */
        img->u = img->y + (size_t)img->stride * img->height;
        img->v = img->u + (size_t)img->uv_stride * (img->height / 2);
    }
}

extern "C" cam_t *cam_open(cam_config *cfg)
{
    /* libcamera is chatty at INFO; keep warnings and errors only unless overridden.
     * V4L2 warns on every start about sensor pixel formats it has no name for. */
    setenv("LIBCAMERA_LOG_LEVELS", "*:WARN,V4L2:ERROR", 0);

    cam_t *c = new cam;
    c->cm = std::make_unique<CameraManager>();
    if (c->cm->start()) {
        fprintf(stderr, "camera: failed to start camera manager\n");
        delete c;
        return nullptr;
    }
    if (c->cm->cameras().empty()) {
        fprintf(stderr, "camera: no camera found (check ribbon cable, then `rpicam-hello --list-cameras`)\n");
        cam_close(c);
        return nullptr;
    }
    c->camera = c->cm->cameras()[0];
    if (c->camera->acquire()) {
        fprintf(stderr, "camera: %s is in use by another process\n", c->camera->id().c_str());
        cam_close(c);
        return nullptr;
    }
    c->acquired = true;

    c->config = c->camera->generateConfiguration({StreamRole::VideoRecording, StreamRole::Viewfinder});
    if (!c->config || c->config->size() != 2) {
        fprintf(stderr, "camera: could not create a two-stream configuration\n");
        cam_close(c);
        return nullptr;
    }
    StreamConfiguration &mc = c->config->at(0);
    StreamConfiguration &lc = c->config->at(1);
    mc.pixelFormat = formats::YUV420;
    mc.size = Size(cfg->width, cfg->height);
    mc.bufferCount = cfg->buffers;
    lc.pixelFormat = formats::YUV420;
    lc.size = Size(cfg->lores_width, cfg->lores_height);
    lc.bufferCount = cfg->buffers;
    /* Full-range BT.601 (as JPEG/JFIF), so the YUV is a known conversion away from the
     * sRGB-like images the model was trained on. */
    mc.colorSpace = ColorSpace::Sycc;
    lc.colorSpace = ColorSpace::Sycc;
    if (cfg->rotate180)
        c->config->orientation = Orientation::Rotate180;

    CameraConfiguration::Status st = c->config->validate();
    if (st == CameraConfiguration::Invalid) {
        fprintf(stderr, "camera: configuration rejected\n");
        cam_close(c);
        return nullptr;
    }
    if (st == CameraConfiguration::Adjusted)
        fprintf(stderr, "camera: configuration adjusted to main %s, lores %s\n",
                mc.toString().c_str(), lc.toString().c_str());
    if (mc.pixelFormat != formats::YUV420 || lc.pixelFormat != formats::YUV420) {
        fprintf(stderr, "camera: YUV420 not supported on this camera\n");
        cam_close(c);
        return nullptr;
    }
    if (lc.colorSpace != ColorSpace::Sycc)
        fprintf(stderr, "camera: warning: lores colour space is %s, not sYCC; colours may be slightly off\n",
                lc.colorSpace ? lc.colorSpace->toString().c_str() : "unset");
    if (c->camera->configure(c->config.get())) {
        fprintf(stderr, "camera: configure failed\n");
        cam_close(c);
        return nullptr;
    }
    cfg->width = mc.size.width;
    cfg->height = mc.size.height;
    cfg->lores_width = lc.size.width;
    cfg->lores_height = lc.size.height;
    c->main_stream = mc.stream();
    c->lores_stream = lc.stream();

    c->alloc = std::make_unique<FrameBufferAllocator>(c->camera);
    for (Stream *s : {c->main_stream, c->lores_stream}) {
        if (c->alloc->allocate(s) < 0) {
            fprintf(stderr, "camera: buffer allocation failed\n");
            cam_close(c);
            return nullptr;
        }
        for (const auto &b : c->alloc->buffers(s))
            if (map_buffer(c, b.get())) {
                cam_close(c);
                return nullptr;
            }
    }

    /* One request per buffer pair; the cookie indexes c->frames. */
    const auto &mb = c->alloc->buffers(c->main_stream);
    const auto &lb = c->alloc->buffers(c->lores_stream);
    size_t n = std::min(mb.size(), lb.size());
    c->frames.resize(n);
    for (size_t i = 0; i < n; i++) {
        std::unique_ptr<Request> r = c->camera->createRequest(i);
        if (!r || r->addBuffer(c->main_stream, mb[i].get()) || r->addBuffer(c->lores_stream, lb[i].get())) {
            fprintf(stderr, "camera: failed to create request\n");
            cam_close(c);
            return nullptr;
        }
        c->requests.push_back(std::move(r));
    }

    c->frame_us = 1000000 / (cfg->fps > 0 ? cfg->fps : 30);
    c->camera->requestCompleted.connect(c, &cam::on_complete);
    fprintf(stderr, "camera: %s, main %dx%d, lores %dx%d, %d fps\n", c->camera->id().c_str(),
            cfg->width, cfg->height, cfg->lores_width, cfg->lores_height, cfg->fps);
    return c;
}

extern "C" int cam_start(cam_t *c)
{
    ControlList ctrls(controls::controls);
    /* Fixing min = max frame duration locks the frame rate; exposure is capped to fit. */
    const int64_t limits[2] = {c->frame_us, c->frame_us};
    ctrls.set(controls::FrameDurationLimits, limits);
    if (c->camera->start(&ctrls)) {
        fprintf(stderr, "camera: start failed\n");
        return -1;
    }
    c->started = true;
    for (auto &r : c->requests)
        if (c->camera->queueRequest(r.get())) {
            fprintf(stderr, "camera: queueRequest failed\n");
            return -1;
        }
    return 0;
}

extern "C" cam_frame *cam_wait_frame(cam_t *c, int timeout_ms)
{
    std::unique_lock<std::mutex> l(c->m);
    if (!c->cv.wait_for(l, std::chrono::milliseconds(timeout_ms), [c] { return !c->done.empty(); }))
        return nullptr;

    /* Keep only the newest frame; give the rest straight back. */
    std::vector<Request *> stale;
    while (c->done.size() > 1) {
        stale.push_back(c->done.front());
        c->done.pop_front();
    }
    Request *r = c->done.front();
    c->done.pop_front();
    c->skipped += stale.size();
    l.unlock();
    for (Request *s : stale)
        c->requeue(s);

    cam_frame *f = &c->frames[r->cookie()];
    fill_image(c, r, c->main_stream, &f->main);
    fill_image(c, r, c->lores_stream, &f->lores);
    const FrameMetadata &md = r->findBuffer(c->main_stream)->metadata();
    f->timestamp_ns = md.timestamp;
    f->sequence = md.sequence;
    f->priv = r;
    return f;
}

extern "C" void cam_release_frame(cam_t *c, cam_frame *f)
{
    if (f && f->priv) {
        Request *r = (Request *)f->priv;
        f->priv = nullptr;
        c->requeue(r);
    }
}

extern "C" unsigned cam_skipped(cam_t *c)
{
    std::lock_guard<std::mutex> l(c->m);
    return c->skipped;
}

extern "C" void cam_close(cam_t *c)
{
    if (!c)
        return;
    if (c->started) {
        c->camera->stop();
        c->started = false;
    }
    if (c->camera)
        c->camera->requestCompleted.disconnect(c);
    c->done.clear();
    c->requests.clear();
    for (auto &mm : c->mmaps)
        munmap(mm.first, mm.second);
    c->mmaps.clear();
    c->mapped.clear();
    if (c->alloc) {
        if (c->main_stream)
            c->alloc->free(c->main_stream);
        if (c->lores_stream)
            c->alloc->free(c->lores_stream);
        c->alloc.reset();
    }
    c->config.reset();
    if (c->acquired)
        c->camera->release();
    c->camera.reset();
    c->cm->stop();
    delete c;
}
