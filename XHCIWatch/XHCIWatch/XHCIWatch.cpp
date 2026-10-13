// XHCIWatch v2 - passive, read-only xHCI fault diagnostic.
// Loads once on IOResources (always matches), then finds every xHCI
// controller (PCI class 0x0c0330) by walking the registry and polls each
// one's PCI Status + USBSTS every 50 ms. Logs the exact fault bits the
// instant a controller errors. Reads only; changes nothing on the hardware.
#include <IOKit/IOService.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <libkern/c++/OSIterator.h>

#define NMAX 8

class com_x_XHCIWatch : public IOService {
    OSDeclareDefaultStructors(com_x_XHCIWatch)
    IOPCIDevice        *fDev[NMAX];
    IOMemoryMap        *fMap[NMAX];
    volatile uint8_t   *fBar[NMAX];
    uint32_t            fOp[NMAX];
    char                fNm[NMAX][16];
    bool                fFired[NMAX];
    int                 fN = 0;
    IOWorkLoop         *fWL  = nullptr;
    IOTimerEventSource *fTmr = nullptr;

    void addDev(IOPCIDevice *d) {
        if (fN >= NMAX) return;
        int i = fN;
        fDev[i] = d; d->retain();
        fFired[i] = false; fMap[i] = nullptr; fBar[i] = nullptr; fOp[i] = 0;
        const char *nm = d->getName();
        snprintf(fNm[i], sizeof(fNm[i]), "%s", nm ? nm : "XHC");
        d->setMemoryEnable(true);
        fMap[i] = d->mapDeviceMemoryWithRegister(0x10);
        if (fMap[i]) { fBar[i] = (volatile uint8_t *)fMap[i]->getVirtualAddress();
                       fOp[i] = *(volatile uint8_t *)fBar[i]; }
        IOLog("XHCIW: watching %s (op=0x%x bar=%p)\n", fNm[i], fOp[i], fBar[i]);
        fN++;
    }

    void poll(IOTimerEventSource *) {
        for (int i = 0; i < fN; i++) {
            IOPCIDevice *d = fDev[i]; if (!d) continue;
            uint16_t sts = d->configRead16(0x06);
            uint32_t usbsts = fBar[i] ? *(volatile uint32_t *)(fBar[i] + fOp[i] + 0x04) : 0;
            bool bad = (sts & 0xF900) || (usbsts & 0x1C);
            if (bad && !fFired[i]) {
                fFired[i] = true;
                uint16_t cmd = d->configRead16(0x04);
                uint16_t cap = (uint16_t)d->findPCICapability(0x10);
                uint16_t devsts = cap ? d->configRead16(cap + 0x0A) : 0;
                IOLog("XHCIW[%s]: FAULT PCISts=0x%04x USBSTS=0x%08x CMD=0x%04x DEVSTS=0x%04x\n",
                      fNm[i], sts, usbsts, cmd, devsts);
                IOLog("XHCIW[%s]:  PCISts: DetParErr=%d SigSysErr=%d RcvMstAbort=%d "
                      "RcvTgtAbort=%d SigTgtAbort=%d MstDataParErr=%d\n", fNm[i],
                      !!(sts&0x8000), !!(sts&0x4000), !!(sts&0x2000),
                      !!(sts&0x1000), !!(sts&0x0800), !!(sts&0x0100));
                IOLog("XHCIW[%s]:  USBSTS: HCHalted=%d HSE=%d HCE=%d | DEVSTS: Corr=%d NonFatal=%d Fatal=%d UR=%d\n",
                      fNm[i], !!(usbsts&1), !!(usbsts&4), !!(usbsts&0x1000),
                      !!(devsts&1), !!(devsts&2), !!(devsts&4), !!(devsts&8));
            }
            if (!bad) fFired[i] = false;
        }
        if (fTmr) fTmr->setTimeoutMS(50);
    }
public:
    virtual bool start(IOService *provider) override {
        if (!IOService::start(provider)) return false;
        // find all xHCI controllers (PCI class 0x0c0330)
        OSDictionary *m = serviceMatching("IOPCIDevice");
        OSIterator *it = m ? getMatchingServices(m) : nullptr;
        if (it) {
            OSObject *o;
            while ((o = it->getNextObject())) {
                IOPCIDevice *d = OSDynamicCast(IOPCIDevice, o);
                if (!d) continue;
                uint32_t cc = d->configRead32(0x08) >> 8;   // class code (24 bits)
                if (cc == 0x0c0330) addDev(d);
            }
            it->release();
        }
        IOLog("XHCIW: found %d xHCI controller(s)\n", fN);
        if (fN == 0) return true;
        fWL = IOWorkLoop::workLoop();
        if (!fWL) return false;
        fTmr = IOTimerEventSource::timerEventSource(this,
                 OSMemberFunctionCast(IOTimerEventSource::Action, this, &com_x_XHCIWatch::poll));
        if (!fTmr || fWL->addEventSource(fTmr) != kIOReturnSuccess) return false;
        fTmr->setTimeoutMS(50);
        registerService();
        return true;
    }
    virtual void stop(IOService *provider) override {
        if (fTmr) { fTmr->cancelTimeout(); if (fWL) fWL->removeEventSource(fTmr); fTmr->release(); fTmr=nullptr; }
        if (fWL) { fWL->release(); fWL=nullptr; }
        for (int i=0;i<fN;i++){ if (fMap[i]) fMap[i]->release(); if (fDev[i]) fDev[i]->release(); }
        IOService::stop(provider);
    }
};

OSDefineMetaClassAndStructors(com_x_XHCIWatch, IOService)
