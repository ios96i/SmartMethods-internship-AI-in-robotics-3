import urllib.request as rq
import bs4 as bs

text = rq.urlopen('http://falhazmi.com/blog/%d8%aa%d8%ad%d9%84%d9%8a%d9%84-%d8%ad%d9%88%d8%a7%d8%af%d8%ab-%d8%a7%d9%84%d9%85%d8%b1%d9%88%d8%b1-%d9%81%d9%8a-%d8%a7%d9%84%d8%b3%d8%b9%d9%88%d8%af%d9%8a%d8%a9/').read()
soup = bs.BeautifulSoup(text,'lxml')

for paragraph in soup.find_all('p'):
    print (paragraph.text)
    

