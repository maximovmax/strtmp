#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <conio.h>
#include <limits.h>
#include <math.h>
#include <memory.h>
#include <float.h>
#include <vector>
#include <list>
#include <dirent.h>
#include <windows.h>
using namespace std;

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#define BYTES_PER_FRAME 8
#define PACKET_SAMPLES 14
#define NIBBLES_PER_FRAME 16
#define PACKET_BYTES 8


struct cdp{
	string path;
	string name;
	bool isfolder;
};

unsigned int soundfiles = 0;
unsigned int patched_sound_files = 0;

inline int DivideByRoundUp(int dividend, int divisor)
{
	return (dividend + divisor - 1) / divisor;
}

inline char GetHighNibble(char value)
{
	return value >> 4 & 0xF;
}

inline char GetLowNibble(char value)
{
	return value & 0xF;
}

inline short Clamp16(int value)
{
	if (value > SHRT_MAX)
		return SHRT_MAX;
	if (value < SHRT_MIN)
		return SHRT_MIN;
	return value;
}

//struct from http://hcs64.com/files/DSPADPCM.us.pdf
typedef struct
{
			// for header generation during decode
		uint32_t num_samples; // total number of RAW samples
		uint32_t num_adpcm_nibbles; // number of ADPCM nibbles (including frame headers)
		uint32_t sample_rate; // Sample rate, in Hz
		
			// DSP addressing and decode context
		uint16_t loop_flag; // 1=LOOPED, 0=NOT LOOPED
		uint16_t format; // Always 0x0000, for ADPCM
		uint32_t sa; // Start offset address for looped samples (zero for non-looped)
		uint32_t ea; // End offset address for looped samples
		uint32_t ca; // always zero
		uint16_t coef[16]; // decode coefficients (eight pairs of 16-bit words)
		
			// DSP decoder initial state
		uint16_t gain; // always zero for ADPCM
		uint16_t ps; // predictor/scale
		uint16_t yn1; // sample history
		uint16_t yn2; // sample history
		
			// DSP decoder loop context
		uint16_t lps; // predictor/scale for loop context
		uint16_t lyn1; // sample history (n-1) for loop context
		uint16_t lyn2; // sample history (n-2) for loop context
		uint16_t pad[11]; // reserved
} sDSPADPCM;

//code from https://github.com/Thealexbarney/DspTool/blob/master/dsptool/decode.c
void decode(uint8_t* src, int16_t* dst, sDSPADPCM* cxt, uint32_t samples)
{
	short hist1 = cxt->yn1;
	short hist2 = cxt->yn2;
	uint16_t* coefs = cxt->coef;
	int frameCount = DivideByRoundUp(samples, PACKET_SAMPLES);
	int samplesRemaining = samples;

	for (int i = 0; i < frameCount; i++)
	{
		int predictor = GetHighNibble(*src);
		int scale = 1 << GetLowNibble(*src++);
		short coef1 = coefs[predictor * 2];
		short coef2 = coefs[predictor * 2 + 1];

		int samplesToRead = MIN(PACKET_SAMPLES, samplesRemaining);

		for (int s = 0; s < samplesToRead; s++)
		{
			int sample = s % 2 == 0 ? GetHighNibble(*src) : GetLowNibble(*src++);
			sample = sample >= 8 ? sample - 16 : sample;
			sample = (((scale * sample) << 11) + 1024 + (coef1 * hist1 + coef2 * hist2)) >> 11;
			short finalSample = Clamp16(sample);

			hist2 = hist1;
			hist1 = finalSample;

			*dst++ = finalSample;
		}

		samplesRemaining -= samplesToRead;
	}
}

//code from https://github.com/jackoalan/gc-dspadpcm-encode/blob/master/grok.c
/* Make sure source includes the yn values (16 samples total) */
void DSPEncodeFrame(short pcmInOut[16], int sampleCount, unsigned char adpcmOut[8], const int16_t coefsIn[8][2])
{
    int inSamples[8][16];
    int outSamples[8][14];

    int bestIndex = 0;

    int scale[8];
    double distAccum[8];

    /* Iterate through each coef set, finding the set with the smallest error */
    for (int i=0 ; i<8 ; i++)
    {
        int v1, v2, v3;
        int distance, index;

        /* Set yn values */
        inSamples[i][0] = pcmInOut[0];
        inSamples[i][1] = pcmInOut[1];

        /* Round and clamp samples for this coef set */
        distance = 0;
        for (int s=0 ; s<sampleCount ; s++)
        {
            /* Multiply previous samples by coefs */
            inSamples[i][s + 2] = v1 = ((pcmInOut[s] * coefsIn[i][1]) + (pcmInOut[s + 1] * coefsIn[i][0])) / 2048;
            /* Subtract from current sample */
            v2 = pcmInOut[s + 2] - v1;
            /* Clamp */
            v3 = (v2 >= 32767) ? 32767 : (v2 <= -32768) ? -32768 : v2;
            /* Compare distance */
            if (abs(v3) > abs(distance))
                distance = v3;
        }

        /* Set initial scale */
        for (scale[i]=0; (scale[i]<=12) && ((distance>7) || (distance<-8)); scale[i]++, distance/=2) {}
        scale[i] = (scale[i] <= 1) ? -1 : scale[i] - 2;

        do
        {
            scale[i]++;
            distAccum[i] = 0;
            index = 0;

            for (int s=0 ; s<sampleCount ; s++)
            {
                /* Multiply previous */
                v1 = ((inSamples[i][s] * coefsIn[i][1]) + (inSamples[i][s + 1] * coefsIn[i][0]));
                /* Evaluate from real sample */
                v2 = ((pcmInOut[s + 2] << 11) - v1) / 2048;
                /* Round to nearest sample */
                v3 = (v2 > 0) ? (int)((double)v2 / (1 << scale[i]) + 0.4999999f) : (int)((double)v2 / (1 << scale[i]) - 0.4999999f);

                /* Clamp sample and set index */
                if (v3 < -8)
                {
                    if (index < (v3 = -8 - v3))
                        index = v3;
                    v3 = -8;
                }
                else if (v3 > 7)
                {
                    if (index < (v3 -= 7))
                        index = v3;
                    v3 = 7;
                }

                /* Store result */
                outSamples[i][s] = v3;

                /* Round and expand */
                v1 = (v1 + ((v3 * (1 << scale[i])) << 11) + 1024) >> 11;
                /* Clamp and store */
                inSamples[i][s + 2] = v2 = (v1 >= 32767) ? 32767 : (v1 <= -32768) ? -32768 : v1;
                /* Accumulate distance */
                v3 = pcmInOut[s + 2] - v2;
                distAccum[i] += v3 * (double)v3;
            }

            for (int x=index+8 ; x>256 ; x>>=1)
                if (++scale[i] >= 12)
                    scale[i] = 11;

        } while ((scale[i] < 12) && (index > 1));
    }

    double min = DBL_MAX;
    for (int i = 0; i < 8; i++)
    {
        if (distAccum[i] < min)
        {
            min = distAccum[i];
            bestIndex = i;
        }
    }

    /* Write converted samples */
    for (int s=0 ; s<sampleCount ; s++)
        pcmInOut[s + 2] = inSamples[bestIndex][s + 2];

    /* Write ps */
    adpcmOut[0] = (char)((bestIndex << 4) | (scale[bestIndex] & 0xF));

    /* Zero remaining samples */
    for (int s=sampleCount ; s<14 ; s++)
        outSamples[bestIndex][s] = 0;

    /* Write output samples */
    for (int y=0; y<7; y++)
    {
        adpcmOut[y + 1] = (char)((outSamples[bestIndex][y * 2] << 4) | (outSamples[bestIndex][y * 2 + 1] & 0xF));
    }
}

static int GetBytesForAdpcmSamples(int samples)
{
    int extraBytes = 0;
    int packets = samples / PACKET_SAMPLES;
    int extraSamples = samples % PACKET_SAMPLES;

    if (extraSamples != 0)
    {
        extraBytes = (extraSamples / 2) + (extraSamples % 2) + 1;
    }

    return PACKET_BYTES * packets + extraBytes;
}

int search_for_file_offset(unsigned char *pptr,int position,int range)
{
	bool bfound = false;
	bool block = false;
	
	for(int x = 0; x < range; x++){
		
		if(pptr[position + x] != 0 && !bfound)
			block = true;
		
		if(pptr[position + x] != 0)
			bfound = true;
		
		else if(block)
				return ((position + x - 4) <= position) ? position: position + x - 4;
					
		
	}
	
	return -1;
}

unsigned int AFS2_offset(unsigned char *pcode,unsigned int psize)
{	
		for(unsigned int x = 0; x < psize; x+= 4)
			if(pcode[x] == 'A' && pcode[x + 1] == 'F' && pcode[x + 2] == 'S' 
				&& pcode[x + 3] == '2')
				return x;
		
		return 0;
}

bool patch_acb_dsp_volume(const char *filename,float value)
{
	FILE *pfile;
	long fsize;
	
	pfile = fopen(filename,"rb");
	
	if(!pfile){
		printf("failed to open file");
		return false;
	}
	
	fseek (pfile , 0 , SEEK_END);
	fsize = ftell (pfile);
	rewind (pfile);
	
	unsigned char *acbbuffer = (unsigned char*)malloc(sizeof(unsigned char)*fsize);

	if(fread(acbbuffer,1,fsize,pfile) != fsize){
		printf("reading error");
		return false;
	}
	fclose(pfile);
	
	//AWB file starts with ASF2
	unsigned int offset = AFS2_offset(acbbuffer,fsize); //FIND AWB HEADER
	
	if(!offset){
		printf("couldn't find header",offset);
		return false;
	}
	
	unsigned char *awbbuffer = acbbuffer + offset; //shift to awb file
	
	
	unsigned int files = *(unsigned int*)&awbbuffer[0x08]; // AWB files +0x08
	unsigned int offset_skip = files*2;

	for(unsigned int x = 0; x < files; x++)
	{
		soundfiles++;
		
		unsigned int address_size = awbbuffer[0x05]; // awb + 0x05 -> offset_size 32bit or 16bit

		unsigned int file_offset = 0;
		printf("Patching file %s[%d]\n",filename,x);
		
		if(address_size == 4)
			file_offset = *(unsigned int*)&awbbuffer[0x10 + offset_skip + (x*4) ];
		else
			file_offset = *(unsigned short*)&awbbuffer[0x10 + offset_skip + (x*2) ];
		
		unsigned int xs = search_for_file_offset(awbbuffer,file_offset,100);
		unsigned int file_offset2 = (xs%0x10) ? xs + (0x10 - ((xs)%0x10)) : xs;
		
		unsigned char *dspbuffer = awbbuffer + file_offset2;
		sDSPADPCM *ptr = (sDSPADPCM*)dspbuffer;

		unsigned int samples = __builtin_bswap32(ptr->num_samples);
		
		if(samples > 1000000) //veery few files error here, guess the file type differs or the offset is incorrect @ file_offset2
			continue;
		
		
		uint16_t* coefs = ptr->coef;
		int16_t coefs2[8][2];
			
		for(unsigned int u = 0; u < 16; u++){
			coefs[u] = __builtin_bswap16(coefs[u]);
		}

		for(unsigned int u = 0; u < 8; u++){
			coefs2[u][0] = coefs[(u*2)];
			coefs2[u][1] = coefs[(u*2)+1];
		}
		
		
		unsigned char *data = dspbuffer + sizeof(sDSPADPCM);
		
		//decode dsp
		int16_t *dest = (int16_t *)malloc(sizeof(int16_t) * samples);
		decode(data,dest,ptr,samples);

		//change volume
		for(int k = 0; k < samples; k++)
			dest[k] = dest[k]*value;
			
		unsigned int packetCount = samples / PACKET_SAMPLES + (samples % PACKET_SAMPLES != 0);
		
		int16_t convSamps[16] = {0};
		unsigned char block[8];
		for (unsigned int p=0 ; p<packetCount ; ++p)
		{
			memset(convSamps + 2, 0, PACKET_SAMPLES * sizeof(int16_t));
			unsigned int numSamples = MIN(samples - p * PACKET_SAMPLES, PACKET_SAMPLES);

			for (unsigned int s=0 ; s<numSamples; ++s)
				convSamps[s+2] = dest[p*PACKET_SAMPLES+s];

			DSPEncodeFrame(convSamps, PACKET_SAMPLES, block, coefs2);

			convSamps[0] = convSamps[14];
			convSamps[1] = convSamps[15];
			
			memcpy(data + (p*8),block,GetBytesForAdpcmSamples(numSamples));
		}
	
		free(dest);	
		patched_sound_files++;
		
		//swap coefs back
		for(int u = 0; u < 16; u++)
				coefs[u] = __builtin_bswap16(coefs[u]);
			
	}
	
	
	FILE *pout = fopen(filename,"wb");
	fwrite(acbbuffer,1,fsize,pout);
	fclose(pout);
	
	
	free(acbbuffer);
	return true;
}


list<string> ReadFolder(string szpath,string Subfolder)
{
    list<string> output;
    DIR *dp;
    dirent *dirp;

    string wPath = szpath + "\\" + Subfolder;

    if( (dp = opendir(wPath.c_str())) == NULL ){
        printf("Filepath incorrect\n");
        return output;
    }

    while( (dirp = readdir(dp)) != NULL){

        if(dirp->d_name[0] == '.' && ( dirp->d_name[1] == '.' || dirp->d_name[1] == 0) )
            continue;

        if(dirp->d_type == 16){
            string subsub = Subfolder + dirp->d_name + "\\";
            list<string> temp = ReadFolder(szpath,subsub);
            output.merge(temp);
        }

        else if(dirp->d_type == 0)
        {
            string Filepath = Subfolder + dirp->d_name;
            output.push_back(Filepath);
        }

    }
    closedir(dp);

    return output;
}

vector<string> getfiles(string path,char (*array)[10],int size)
{
    vector<string> output;

    list<string> file_list = ReadFolder(path,"");
    if(file_list.empty())
    	return output;

    file_list.sort();

    for(list<string>::iterator i = file_list.begin(); i != file_list.end(); i++)
    {
        bool btemp = false;

        for(int x = 0; x < size; x++)
        {
            if( (*i).rfind(array[x]) != string::npos){
                btemp = true;
                break;
            }
        }
        if(btemp)
            output.push_back(*i);
    }

    file_list.clear();

    return output;
}

int main()
{
	
	char szpath[MAX_PATH];
	GetCurrentDirectoryA(MAX_PATH,szpath);
	
	string cpath;
	cpath += szpath;
	cpath += "\\se\\";
	
	float multiplicator = 1.0f;
	
	printf("This program changes the volume of every acb file inside the \"se\" folder\n");
	printf("Make sure you have a backup copy of the folder before you proceed as it overrides the files\n");
	printf("Accepted value 0.0 - 2.0 (1.0 is default for no patching)\n\n");
	printf("Enter volume multiplicator: ");
	
	scanf("%f",&multiplicator);
	
	if(multiplicator == 1.0f){
		printf("No patching\n");
		return 1;
	}
	
	if(multiplicator < 0.0f)
		multiplicator = 0.0f;
	if(multiplicator > 2.0f)
		multiplicator = 2.0f;
	
	char array[][10] = {"acb"};
    vector<string> lfiles = getfiles(cpath,array,1);
	
	for(unsigned int x = 0; x < lfiles.size(); x++){
		
		string filepath = cpath + lfiles[x];
		patch_acb_dsp_volume(filepath.c_str(),multiplicator);
	}
	
	printf("\nSuccessfully patched %d of %d sound files (%f%%)\nPress any key to exit",patched_sound_files,soundfiles,(float(patched_sound_files)/float(soundfiles))*100.0f );
	getch();
	
	return 0;
}