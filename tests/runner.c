/* 测试用的最小驱动：读一张 pgm，跑一遍完整流水线，
   把二值图和最终中心线写出来，交给 python 侧比对。
   不依赖 OpenCV，所以能脱离整车环境单独编译。 */
#include <stdio.h>
#include "image.h"

int main(int argc, char **argv)
{
    if (argc < 4) {
        printf("用法: %s 输入.pgm 二值输出.pgm 中心线.txt\n", argv[0]);
        return 1;
    }

    read_pgm(argv[1]);
    process_current_frame();
    save_pgm(argv[2], bin_image);

    FILE *fp = fopen(argv[3], "w");
    if (!fp) return 1;
    for (int y = 0; y < IMAGE_H; y++)
        fprintf(fp, "%d %d\n", y, center_line_final[y]);   // -1 表示这一行没有测量值
    fclose(fp);
    return 0;
}
