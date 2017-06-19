
class utils:
    """Resize keeping aspect ratio"""
    @staticmethod
    def getsize(sz, screensize):
        # check if the current width is larger than the max
        srcWidth = float(sz[0])
        srcHeight = float(sz[1])
        maxWidth = screensize[0]
        maxHeight = screensize[1]

        ratio = min(maxWidth / srcWidth, maxHeight / srcHeight)

        return {"size": (int(srcWidth * ratio), int(srcHeight * ratio)),
                "scale": ratio}

    """Calc offset"""
    @staticmethod
    def getposition(sz, screensize):
        return ((screensize[0]-sz[0])/2,(screensize[1]-sz[1])/2)

    """Convert point into window coords"""
    @staticmethod
    def convertpoint(pt, offset, scale):
        return ((pt[0]-offset[0])/scale, (pt[1]-offset[1])/scale)
