from dataclasses import dataclass, field
from typing import List, Dict, Optional

'''
ADC 数据结构体
'''
# TODO: 这逼玩意儿里面是啥？
@dataclass
class adcStruct:
    adc: List[float] = field(default_factory=list)
    ampPhaFactor: List[float] = field(default_factory=list)
    virtualArrayLocation: List[float] = field(default_factory=list)


'''
BV 数据结构体
'''
@dataclass
class beamVectorStruct:
    antSpaceFactorMode1: float
    antSpaceFactorMode2: float
    # TODO: Rx 天线怎么算的？等效 24 个接收天线？
    beamVector: List[Dict]
    beamVectorMode: List[int]
    chirpFreqStartMode1: float
    chirpFreqStartMode2: float
    virtualArrayLocationBinMode1: int
    virtualArrayLocationBinMode2: int

'''
帧信息结构体
'''
# FIXME: 可能存在数据类型错误问题导致数据解析过程中强制类型转换出错 
@dataclass
class TLVHeader:
    DataChecksum: int
    Length: int
    Type: int

@dataclass
class frameHeader:
    FrameHeaderChecksum: int
    FrameNumber: int
    MagicWord: int
    SubframeNumber: int
    Time: int
    TotalPacketLength: int
    Version: int
    angleResolution: float
    frameRealTime_ms: int
    frameRealTime_s: int
    numAngleBins: int
    numDetObj: int
    numDopplerBins: int
    numDopplerBinsADC: int
    numHrrpMaxLength: int
    numRangeBins: int
    numRxAntennas: int
    numTLVs: int
    numTracker: int
    numTxAntennas: int
    numVirtualAntennas: int
    radarAltitude: int
    radarDeflectionAngle: int
    radarLatitude: int
    radarLongitude: int
    radarNorthAngle: int
    rangeResolutionHigh: float
    rangeResolutionLow: float
    reserved: int
    tLVHeaderlist: List[TLVHeader]
    velResolution: float
    add_frameTime_ms: int


'''
HRRP 数据结构体
'''
@dataclass
class hrrpStruct:
    # TODO: 1736500679_876.json 文件中的 HRRP 数据总共包含 21120 个数据，当前帧的点迹数为 165，则 21120 / 165 = 128 有什么意义
    hrrp: List[float]


'''
RD 索引数据结构体
'''
@dataclass
class rangeDopplerStruct:
    # TODO: 根据距离维分辨单元数 512 和多普勒维分辨单元数 512 可以计算出总共有 262144 个值
    rangeDoppler: List[float]


'''
RA 索引数据结构体
'''
@dataclass
class rangeAngleStruct:
    # TODO: 怎么你妈逼又是空的？
    angleaxis: List[float] = field(default_factory=list)
    ra: List[float] = field(default_factory=list)
    rangeaxis: List[float] = field(default_factory=list)


'''
点迹数据结构体
'''
@dataclass
class detections:
    angle: float
    doppler: float
    dopplerIdx: int
    peakVal: float
    range: float
    rangeIdx: int
    snr: float
    add_pos_x: float = None
    add_pos_y: float = None 

@dataclass
class detPointsStruct:
    dets: List[detections]


'''
航迹数据结构体
'''
@dataclass
class tracks:
    RCS: int
    ax: int
    ay: int
    confidence: int
    lane: int
    reserved1: int
    reserved2: int
    targetClass: int
    trackId: int
    vx: int
    vy: int
    x: int
    xSize: int
    y: int
    ySize: int
    zSize: int

@dataclass
class trackStruct:
    trks: List[tracks]


'''
雷达数据结构体
'''
@dataclass
class Radar:
    adc: Optional[adcStruct] = None
    bv: Optional[beamVectorStruct] = None
    frameInfo: Optional[frameHeader] = None
    hrrp: Optional[hrrpStruct] = None
    rd: Optional[rangeDopplerStruct] = None
    ra: Optional[rangeAngleStruct] = None
    dets: Optional[detPointsStruct] = None
    trks: Optional[trackStruct] = None
