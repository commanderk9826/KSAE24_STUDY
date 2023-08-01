import urllib.request
import urllib.parse
import json

url ="http://date.jsontest.com"
response =  urllib.request.urlopen(url)
data = json.loads(response.read())
print(data)