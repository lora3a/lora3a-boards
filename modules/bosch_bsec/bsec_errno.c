#include "bsec_errno.h"

char* bsec_errno(bsec_library_return_t err)
{
    switch(err) {
        case BSEC_OK:
            return "OK";
        case BSEC_E_DOSTEPS_INVALIDINPUT:
            return "E_DOSTEPS_INVALIDINPUT";
        case BSEC_E_DOSTEPS_VALUELIMITS:
            return "E_DOSTEPS_VALUELIMITS";
        case BSEC_W_DOSTEPS_TSINTRADIFFOUTOFRANGE:
            return "W_DOSTEPS_TSINTRADIFFOUTOFRANGE";
        case BSEC_E_DOSTEPS_DUPLICATEINPUT:
            return "E_DOSTEPS_DUPLICATEINPUT";
        case BSEC_I_DOSTEPS_NOOUTPUTSRETURNABLE:
            return "I_DOSTEPS_NOOUTPUTSRETURNABLE";
        case BSEC_W_DOSTEPS_EXCESSOUTPUTS:
            return "W_DOSTEPS_EXCESSOUTPUTS";
        case BSEC_W_DOSTEPS_GASINDEXMISS:
            return "W_DOSTEPS_GASINDEXMISS";
        case BSEC_E_SU_WRONGDATARATE:
            return "E_SU_WRONGDATARATE";
        case BSEC_E_SU_SAMPLERATELIMITS:
            return "E_SU_SAMPLERATELIMITS";
        case BSEC_E_SU_DUPLICATEGATE:
            return "E_SU_DUPLICATEGATE";
        case BSEC_E_SU_INVALIDSAMPLERATE:
            return "E_SU_INVALIDSAMPLERATE";
        case BSEC_E_SU_GATECOUNTEXCEEDSARRAY:
            return "E_SU_GATECOUNTEXCEEDSARRAY";
        case BSEC_E_SU_SAMPLINTVLINTEGERMULT:
            return "E_SU_SAMPLINTVLINTEGERMULT";
        case BSEC_E_SU_MULTGASSAMPLINTVL:
            return "E_SU_MULTGASSAMPLINTVL";
		case BSEC_E_SU_HIGHHEATERONDURATION:
		    return "E_SU_HIGHHEATERONDURATION";
		case BSEC_W_SU_UNKNOWNOUTPUTGATE:
		    return "W_SU_UNKNOWNOUTPUTGATE";
		case BSEC_W_SU_MODINNOULP:
		    return "W_SU_MODINNOULP";
		case BSEC_I_SU_SUBSCRIBEDOUTPUTGATES:
		    return "I_SU_SUBSCRIBEDOUTPUTGATES";
		case BSEC_I_SU_GASESTIMATEPRECEDENCE:
		    return "I_SU_GASESTIMATEPRECEDENCE";
		case BSEC_W_SU_SAMPLERATEMISMATCH:
		    return "W_SU_SAMPLERATEMISMATCH";
		case BSEC_E_PARSE_SECTIONEXCEEDSWORKBUFFER:
		    return "E_PARSE_SECTIONEXCEEDSWORKBUFFER";
		case BSEC_E_CONFIG_FAIL:
		    return "E_CONFIG_FAIL";
		case BSEC_E_CONFIG_VERSIONMISMATCH:
		    return "E_CONFIG_VERSIONMISMATCH";
		case BSEC_E_CONFIG_FEATUREMISMATCH:
		    return "E_CONFIG_FEATUREMISMATCH";
		case BSEC_E_CONFIG_CRCMISMATCH:
		    return "E_CONFIG_CRCMISMATCH";
		case BSEC_E_CONFIG_EMPTY:
		    return "E_CONFIG_EMPTY";
		case BSEC_E_CONFIG_INSUFFICIENTWORKBUFFER:
		    return "E_CONFIG_INSUFFICIENTWORKBUFFER";
		case BSEC_E_CONFIG_INVALIDSTRINGSIZE:
		    return "E_CONFIG_INVALIDSTRINGSIZE";
		case BSEC_E_CONFIG_INSUFFICIENTBUFFER:
		    return "E_CONFIG_INSUFFICIENTBUFFER";
		case BSEC_E_SET_INVALIDCHANNELIDENTIFIER:
		    return "E_SET_INVALIDCHANNELIDENTIFIER";
		case BSEC_E_SET_INVALIDLENGTH:
		    return "E_SET_INVALIDLENGTH";
		case BSEC_W_SC_CALL_TIMING_VIOLATION:
		    return "W_SC_CALL_TIMING_VIOLATION";
		case BSEC_W_SC_MODEXCEEDULPTIMELIMIT:
		    return "W_SC_MODEXCEEDULPTIMELIMIT";
		case BSEC_W_SC_MODINSUFFICIENTWAITTIME:
		    return "W_SC_MODINSUFFICIENTWAITTIME";
        default:
            return "INVALID ERRNO";
    }
    return "INVALID ERRNO";
}
